"""
Cooperative agent module for CARLA + CoPerception integration (TASK-01, Phase 1).

This module defines CooperativeAgent, a subclass of carla.agents.navigation.BehaviorAgent
whose hazard-detection logic is replaced by fused 3D bounding boxes produced by
CoPerception. No privileged CARLA ground-truth data is used for driving decisions.

Responsibilities:
    1. Inherit the PID controller and waypoint tracker from BehaviorAgent.
    2. At each simulation step, read local sensor tensors (LiDAR + camera).
    3. Run CoPerception feature extraction on local data.
    4. Fuse local features with cooperative features received from partner vehicles
       (shared in-memory registry for Phase 1; OMNeT++ network in Phase 2).
    5. Produce fused 3D bounding boxes in the ego vehicle reference frame.
    6. Evaluate collision risk (Time-to-Collision, obstacle on trajectory) using
       ONLY the fused bounding boxes.
    7. Apply closed-loop control: emergency brake, reduced target speed, or increased
       TTC margin when perception indicates danger.
    8. If perception is missing or degraded, react physically (late braking or
       collision) to demonstrate the closed-loop safety property.

Hard constraints respected:
    - Driving decisions rely strictly on CoPerception fused bounding boxes.
    - No privileged CARLA actor positions are queried for hazard detection.
"""

from __future__ import annotations

import logging
import sys
from typing import Any, Dict, List, Optional, Tuple

# Ensure CARLA PythonAPI carla/ package is importable so that 'agents'
# is available as a top-level package (behavior_agent imports
# 'from agents.navigation.basic_agent import BasicAgent').
# Use append (not insert(0)) so that 'carla' (with Client, Vehicle, etc.)
# is resolved from the venv site-packages first.
_CARLA_PYTHON_API = "/home/alberto/carla/PythonAPI/carla"
if _CARLA_PYTHON_API not in sys.path:
    sys.path.append(_CARLA_PYTHON_API)

import carla  # type: ignore
import numpy as np
import torch  # type: ignore
from agents.navigation.behavior_agent import BehaviorAgent  # type: ignore

logger = logging.getLogger(__name__)

# Global cooperative detection registry (Phase 1: in-memory sharing).
# In Phase 2 this will be replaced by OMNeT++ network messages.
# Structure: { vehicle_carla_id: {"boxes": np.ndarray[N, 7], "timestamp": float} }
# Each box: [x, y, z, dx, dy, dz, yaw] in world coordinates.
_cooperative_registry: Dict[int, Dict[str, Any]] = {}


def register_detection(vehicle_id: int, boxes: np.ndarray, timestamp: float) -> None:
    """Publish fused 3D bounding boxes to the cooperative registry.

    This allows partner vehicles to access this vehicle's detections.
    In Phase 2, this will be replaced by OMNeT++ network transmission.

    Args:
        vehicle_id: CARLA id of the publishing vehicle.
        boxes: Numpy array of shape [N, 7] with [x, y, z, dx, dy, dz, yaw].
        timestamp: Simulation timestamp of the detection.
    """
    _cooperative_registry[vehicle_id] = {
        "boxes": boxes,
        "timestamp": timestamp,
    }
    logger.debug(
        "Registered detection for vehicle %d: %d boxes at t=%.3f",
        vehicle_id,
        len(boxes) if boxes is not None else 0,
        timestamp,
    )


def get_cooperative_boxes(vehicle_id: int, exclude_self: bool = True) -> Dict[int, Dict[str, Any]]:
    """Retrieve cooperative detections from other vehicles.

    Args:
        vehicle_id: CARLA id of the requesting vehicle (excluded from results).
        exclude_self: Whether to exclude the requesting vehicle's own boxes.

    Returns:
        Dictionary mapping partner vehicle id -> detection data.
    """
    result: Dict[int, Dict[str, Any]] = {}
    for vid, data in _cooperative_registry.items():
        if exclude_self and vid == vehicle_id:
            continue
        result[vid] = data
    return result


class CooperativeAgent(BehaviorAgent):
    """BehaviorAgent subclass that drives using CoPerception fused bounding boxes
    instead of privileged CARLA ground-truth.

    The agent overrides the hazard-detection step of BehaviorAgent so that
    collision risk is evaluated exclusively on fused 3D boxes produced by
    CoPerception (local + cooperative). If perception is degraded or missing,
    the agent reacts physically (emergency brake or collision) to demonstrate
    the closed-loop safety property.
    """

    def __init__(
        self,
        vehicle: carla.Vehicle,
        target_speed: float = 20.0,
        coop_partner_ids: Optional[List[int]] = None,
        ttc_threshold: float = 3.0,
        enable_cooperation: bool = True,
        **kwargs: Any,
    ) -> None:
        """Initialize the CooperativeAgent.

        Args:
            vehicle: The CARLA vehicle actor this agent controls.
            target_speed: Desired target speed in km/h.
            coop_partner_ids: List of CARLA vehicle ids considered as cooperative
                              partners (their boxes are fused with local ones).
            ttc_threshold: Time-to-Collision threshold in seconds below which
                           emergency braking is triggered.
            enable_cooperation: Whether to fuse cooperative boxes from partners.
            **kwargs: Additional arguments forwarded to BehaviorAgent.
        """
        super().__init__(vehicle, behavior='normal', **kwargs)

        # Set target speed after initialization (BehaviorAgent exposes
        # set_target_speed in CARLA 0.9.15)
        self.set_target_speed(target_speed)

        self.coop_partner_ids: List[int] = coop_partner_ids or []
        self.ttc_threshold = ttc_threshold
        self.enable_cooperation = enable_cooperation

        # Latest fused boxes from CoPerception (world coordinates)
        self.latest_fused_boxes: np.ndarray = np.empty((0, 7), dtype=np.float32)
        self.last_detection_timestamp: float = 0.0

        # Perception quality tracking (for Konro feedback in Phase 3)
        self.perception_available: bool = False
        self.consecutive_missing_frames: int = 0

        logger.info(
            "CooperativeAgent initialized for vehicle %d (target_speed=%.1f km/h, "
            "ttc_threshold=%.1fs, cooperation=%s)",
            vehicle.id,
            target_speed,
            ttc_threshold,
            enable_cooperation,
        )

    # ------------------------------------------------------------------
    # Perception and fusion
    # ------------------------------------------------------------------

    def update_perception(
        self,
        local_boxes: np.ndarray,
        timestamp: float,
    ) -> None:
        """Update the agent's perception with locally detected boxes and fuse
        cooperative boxes from partners.

        Args:
            local_boxes: Numpy array [N, 7] of locally detected 3D boxes
                         [x, y, z, dx, dy, dz, yaw] in world coordinates.
            timestamp: Simulation timestamp of the detection.
        """
        self.last_detection_timestamp = timestamp
        self.perception_available = local_boxes is not None and len(local_boxes) > 0

        if self.perception_available:
            self.consecutive_missing_frames = 0
            all_boxes = [local_boxes]
        else:
            self.consecutive_missing_frames += 1
            all_boxes = []

        # Fuse cooperative boxes from partners
        if self.enable_cooperation:
            partner_boxes = get_cooperative_boxes(self._vehicle.id, exclude_self=True)
            for partner_id, data in partner_boxes.items():
                if partner_id in self.coop_partner_ids or not self.coop_partner_ids:
                    partner_boxes_arr = data.get("boxes", None)
                    if partner_boxes_arr is not None and len(partner_boxes_arr) > 0:
                        all_boxes.append(partner_boxes_arr)
                        logger.debug(
                            "Fused %d cooperative boxes from vehicle %d",
                            len(partner_boxes_arr),
                            partner_id,
                        )

        if all_boxes:
            self.latest_fused_boxes = np.vstack(all_boxes)
        else:
            self.latest_fused_boxes = np.empty((0, 7), dtype=np.float32)

        # Publish this vehicle's fused boxes to the cooperative registry
        register_detection(self._vehicle.id, self.latest_fused_boxes, timestamp)

    def compute_ttc(
        self,
        box: np.ndarray,
        ego_velocity: carla.Vector3D,
    ) -> float:
        """Compute Time-to-Collision (TTC) with a single detected box.

        Uses a simplified constant-velocity model: TTC = distance /
        relative_speed along the line of sight. Returns infinity if the
        object is moving away or relative speed is negligible.

        Args:
            box: 1D array [x, y, z, dx, dy, dz, yaw] in world coordinates.
            ego_velocity: Ego vehicle velocity vector.

        Returns:
            TTC in seconds, or float('inf') if no collision course is detected.
        """
        box_pos = np.array([box[0], box[1], box[2]], dtype=np.float64)
        ego_pos = np.array(
            [
                self._vehicle.get_location().x,
                self._vehicle.get_location().y,
                self._vehicle.get_location().z,
            ],
            dtype=np.float64,
        )

        relative_pos = box_pos - ego_pos
        distance = float(np.linalg.norm(relative_pos))

        if distance < 0.1:
            return 0.0

        ego_speed = float(np.sqrt(
            ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2
        ))

        if ego_speed < 0.1:
            return float("inf")

        # Unit vector from ego to object
        los = relative_pos / distance

        # Relative speed along line of sight (positive = approaching)
        ego_vel_vec = np.array([ego_velocity.x, ego_velocity.y, ego_velocity.z], dtype=np.float64)
        approach_speed = float(np.dot(ego_vel_vec, los))

        if approach_speed <= 0.1:
            return float("inf")

        ttc = distance / approach_speed
        return max(0.0, ttc)

    def detect_hazards(
        self,
        ego_velocity: carla.Vector3D,
    ) -> Tuple[bool, Optional[np.ndarray], float]:
        """Detect collision hazards using ONLY fused CoPerception bounding boxes.

        This replaces BehaviorAgent's ground-truth-based hazard detection.
        No privileged CARLA actor data is consulted.

        Args:
            ego_velocity: Current ego vehicle velocity vector.

        Returns:
            Tuple of (hazard_detected, closest_box, min_ttc).
        """
        if not self.perception_available or len(self.latest_fused_boxes) == 0:
            # Perception missing: cannot confirm safety, treat as potential hazard
            logger.warning(
                "Vehicle %d: no perception data available (missing frames=%d)",
                self._vehicle.id,
                self.consecutive_missing_frames,
            )
            return True, None, 0.0

        min_ttc = float("inf")
        closest_box: Optional[np.ndarray] = None
        hazard_detected = False

        for box in self.latest_fused_boxes:
            ttc = self.compute_ttc(box, ego_velocity)
            if ttc < min_ttc:
                min_ttc = ttc
                closest_box = box

            if ttc < self.ttc_threshold:
                hazard_detected = True

        return hazard_detected, closest_box, min_ttc

    # ------------------------------------------------------------------
    # Control override
    # ------------------------------------------------------------------

    def run_step(self, target_speed: Optional[float] = None) -> carla.VehicleControl:
        """Execute one control step using CoPerception-based hazard detection.

        Overrides BehaviorAgent.run_step() so that hazard evaluation relies
        exclusively on fused bounding boxes rather than privileged ground truth.

        Args:
            target_speed: Optional target speed override in km/h.

        Returns:
            A carla.VehicleControl command.
        """
        if target_speed is not None:
            self._target_speed = target_speed

        # Get current velocity
        velocity = self._vehicle.get_velocity()

        # Detect hazards using CoPerception fused boxes only
        hazard_detected, closest_box, min_ttc = self.detect_hazards(velocity)

        # Closed-loop reaction
        if hazard_detected:
            if min_ttc < self.ttc_threshold * 0.5:
                # Imminent collision: emergency brake
                logger.warning(
                    "Vehicle %d: EMERGENCY BRAKE (TTC=%.2fs < %.2fs)",
                    self._vehicle.id,
                    min_ttc,
                    self.ttc_threshold * 0.5,
                )
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                control.hand_brake = False
                control.steer = 0.0
                return control
            else:
                # Approaching hazard: reduce speed and increase safety margin
                logger.info(
                    "Vehicle %d: reducing speed (TTC=%.2fs)",
                    self._vehicle.id,
                    min_ttc,
                )
                # Delegate to BehaviorAgent but with reduced target speed.
                # BehaviorAgent.run_step() takes no arguments; target speed
                # is read from self._target_speed, which we set directly.
                reduced_speed = max(5.0, self._target_speed * 0.5)
                self._target_speed = reduced_speed
                return super().run_step()
        else:
            # No hazard detected: proceed normally
            self.consecutive_missing_frames = 0
            return super().run_step()

    def get_perception_status(self) -> Dict[str, Any]:
        """Return a dictionary with perception quality metrics.

        Useful for Konro feedback (Phase 3) and experimental analysis.

        Returns:
            Dictionary with keys: available, consecutive_missing_frames,
            num_fused_boxes, last_timestamp.
        """
        return {
            "available": self.perception_available,
            "consecutive_missing_frames": self.consecutive_missing_frames,
            "num_fused_boxes": int(len(self.latest_fused_boxes)),
            "last_timestamp": self.last_detection_timestamp,
        }
