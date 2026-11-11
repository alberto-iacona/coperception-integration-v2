"""
Cooperative agent module for CARLA + CoPerception integration (TASK-02, Phase 2).

This module defines CooperativeAgent, a subclass of carla.agents.navigation.BehaviorAgent
whose hazard-detection logic is replaced by fused 3D bounding boxes produced by
CoPerception and exchanged through OMNeT++ network simulation. No privileged
CARLA ground-truth data is used for driving decisions.

Responsibilities:
    1. Inherit the PID controller and waypoint tracker from BehaviorAgent.
    2. At each simulation step, read local sensor tensors (LiDAR + camera).
    3. Run CoPerception feature extraction on local data.
    4. Publish local fused boxes to the OMNeT++ bridge for partner retrieval.
    5. Request partner boxes through OMNeT++ (deliver/delay decided by network simulation).
    6. Apply network effects: packet loss (zeros/oldest buffer) and delay
       (circular buffer temporal indexing).
    7. Produce fused 3D bounding boxes in the ego vehicle reference frame.
    8. Evaluate collision risk (Time-to-Collision, obstacle on trajectory) using
       ONLY the fused bounding boxes received through OMNeT++.
    9. Apply closed-loop control: emergency brake, reduced target speed, or
       increased TTC margin when perception indicates danger.
    10. If perception is missing or network fails, react physically (late braking
        or collision) to demonstrate the closed-loop safety property.

Hard constraints respected:
    - Driving decisions rely strictly on CoPerception fused bounding boxes
      delivered through OMNeT++.
    - No privileged CARLA actor positions are queried for hazard detection.
"""

from __future__ import annotations

import logging
import sys
from typing import Any, Dict, List, Optional, Tuple

# Ensure CARLA PythonAPI carla/ package is importable so that 'agents'
# is available as a top-level package (behavior_agent imports
# 'from agents.navigation.basic_agent import BasicAgent').
# This MUST be done BEFORE importing carla and behavior_agent.
_CARLA_PYTHON_API = "/home/alberto/carla/PythonAPI/carla"
if _CARLA_PYTHON_API not in sys.path:
    sys.path.append(_CARLA_PYTHON_API)

import carla  # type: ignore
import numpy as np
import torch  # type: ignore
from agents.navigation.behavior_agent import BehaviorAgent  # type: ignore

logger = logging.getLogger(__name__)


class CooperativeAgent(BehaviorAgent):
    """BehaviorAgent subclass that drives using CoPerception fused bounding boxes
    exchanged through OMNeT++ network simulation instead of privileged
    CARLA ground-truth.

    The agent overrides the hazard-detection step of BehaviorAgent so that
    collision risk is evaluated exclusively on fused 3D boxes produced by
    CoPerception and delivered through OMNeT++ (with realistic packet loss
    and delay). If perception is degraded or the network fails, the agent
    reacts physically (emergency brake or collision) to demonstrate the
    closed-loop safety property.
    """

    def __init__(
        self,
        vehicle: carla.Vehicle,
        bridge: Any,
        target_speed: float = 20.0,
        coop_partner_ids: Optional[List[int]] = None,
        ttc_threshold: float = 3.0,
        enable_cooperation: bool = True,
        dataset_framerate: float = 5.0,
        max_buffer_sec: float = 2.0,
        **kwargs: Any,
    ) -> None:
        """Initialize the CooperativeAgent.

        Args:
            vehicle: The CARLA vehicle actor this agent controls.
            bridge: OmnetBridge instance for network communication.
            target_speed: Desired target speed in km/h.
            coop_partner_ids: List of CARLA vehicle ids considered as cooperative
                              partners (their boxes are fused with local ones).
            ttc_threshold: Time-to-Collision threshold in seconds below which
                           emergency braking is triggered.
            enable_cooperation: Whether to fuse cooperative boxes from partners
                                through OMNeT++.
            dataset_framerate: Dataset/simulation framerate in Hz, used to
                               convert network delay to frame lag.
            max_buffer_sec: Maximum buffer length in seconds for partner boxes.
            **kwargs: Additional arguments forwarded to BehaviorAgent.
        """
        super().__init__(vehicle, behavior="normal", **kwargs)

        # Set target speed after initialization (BehaviorAgent exposes
        # set_target_speed in CARLA 0.9.15)
        self.set_target_speed(target_speed)

        self.bridge = bridge
        self.coop_partner_ids: List[int] = coop_partner_ids or []
        self.ttc_threshold = ttc_threshold
        self.enable_cooperation = enable_cooperation
        self.dataset_framerate = dataset_framerate

        # Circular buffer of partner boxes per partner vehicle id
        # Structure: { partner_id: [(timestamp, boxes), ...] }
        self._partner_boxes_buffer: Dict[int, List[Tuple[float, np.ndarray]]] = {}
        self._max_buffer_len = int(max_buffer_sec * dataset_framerate)

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
    # Perception and fusion through OMNeT++
    # ------------------------------------------------------------------

    def update_perception(
        self,
        local_boxes: np.ndarray,
        timestamp: float,
    ) -> None:
        """Update the agent's perception with locally detected boxes and fuse
        cooperative boxes received through OMNeT++.

        Args:
            local_boxes: Numpy array [N, 7] of locally detected 3D boxes
                         [x, y, z, dx, dy, dz, yaw] in world coordinates.
            timestamp: Simulation timestamp of the detection.
        """
        self.last_detection_timestamp = timestamp

        # Publish local boxes to OMNeT++ bridge so partners can retrieve them
        if (
            self.bridge is not None
            and local_boxes is not None
            and len(local_boxes) > 0
        ):
            self.bridge.publish_boxes(
                vehicle_id=self._vehicle.id,
                boxes=local_boxes,
                timestamp=timestamp,
            )

        # Determine perception availability
        self.perception_available = local_boxes is not None and len(local_boxes) > 0

        if self.perception_available:
            self.consecutive_missing_frames = 0
            all_boxes = [local_boxes]
        else:
            self.consecutive_missing_frames += 1
            all_boxes = []

        # Fuse cooperative boxes from partners through OMNeT++
        if self.enable_cooperation and self.bridge is not None:
            partner_ids = (
                self.coop_partner_ids
                if self.coop_partner_ids
                else [
                    p
                    for p in self.bridge._published_boxes.keys()
                    if p != self._vehicle.id
                ]
            )
            for partner_id in partner_ids:
                partner_boxes, delay_s, delivered = self.bridge.get_partner_boxes(
                    receiver_id=self._vehicle.id,
                    partner_id=partner_id,
                )
                if (
                    delivered
                    and partner_boxes is not None
                    and len(partner_boxes) > 0
                ):
                    # Store received partner boxes in buffer for future delay lookups
                    self.store_partner_boxes(partner_id, partner_boxes, timestamp)

                    # Apply network delay: retrieve buffered box from the past
                    delayed_boxes = self._get_delayed_boxes(
                        partner_id, partner_boxes, delay_s, timestamp
                    )
                    if delayed_boxes is not None and len(delayed_boxes) > 0:
                        all_boxes.append(delayed_boxes)
                        logger.debug(
                            "Fused %d cooperative boxes from vehicle %d "
                            "(delay=%.3fs)",
                            len(delayed_boxes),
                            partner_id,
                            delay_s,
                        )
                    else:
                        # Buffer underflow: use most recent available
                        all_boxes.append(partner_boxes)
                        logger.debug(
                            "Fused %d cooperative boxes from vehicle %d "
                            "(buffer underflow, using most recent)",
                            len(partner_boxes),
                            partner_id,
                        )
                elif not delivered:
                    # Packet lost: skip this partner's data for this frame
                    logger.debug(
                        "Packet lost from vehicle %d (no fusion)",
                        partner_id,
                    )

        if all_boxes:
            self.latest_fused_boxes = np.vstack(all_boxes)
        else:
            self.latest_fused_boxes = np.empty((0, 7), dtype=np.float32)

    def _get_delayed_boxes(
        self,
        partner_id: int,
        current_boxes: np.ndarray,
        delay_s: float,
        current_timestamp: float,
    ) -> Optional[np.ndarray]:
        """Retrieve partner boxes from the circular buffer at the delayed timestamp.

        Args:
            partner_id: Partner vehicle ID.
            current_boxes: Most recent boxes from the partner (fallback).
            delay_s: Network delay in seconds.
            current_timestamp: Current simulation timestamp.

        Returns:
            Delayed boxes from the buffer, or None if insufficient history.
        """
        if delay_s <= 0.0:
            return current_boxes

        target_timestamp = current_timestamp - delay_s
        buffer = self._partner_boxes_buffer.get(partner_id, [])

        if not buffer:
            return None

        # Find the box entry closest to target_timestamp (not after it)
        best_entry = None
        best_diff = float("inf")
        for ts, boxes in buffer:
            if ts <= target_timestamp:
                diff = target_timestamp - ts
                if diff < best_diff:
                    best_diff = diff
                    best_entry = boxes

        if best_entry is not None:
            return best_entry

        # Buffer underflow: return the oldest available if within acceptable range
        if buffer:
            oldest_ts, oldest_boxes = buffer[0]
            if current_timestamp - oldest_ts <= delay_s + 1.0:
                return oldest_boxes

        return None

    def store_partner_boxes(
        self,
        partner_id: int,
        boxes: np.ndarray,
        timestamp: float,
    ) -> None:
        """Store partner boxes in the circular buffer for delayed retrieval.

        Args:
            partner_id: Partner vehicle ID.
            boxes: Numpy array [N, 7] of partner boxes.
            timestamp: Simulation timestamp of the boxes.
        """
        if partner_id not in self._partner_boxes_buffer:
            self._partner_boxes_buffer[partner_id] = []

        self._partner_boxes_buffer[partner_id].append((timestamp, boxes.copy()))

        # Maintain fixed buffer length
        if len(self._partner_boxes_buffer[partner_id]) > self._max_buffer_len:
            self._partner_boxes_buffer[partner_id].pop(0)

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
        ego_vel_vec = np.array(
            [ego_velocity.x, ego_velocity.y, ego_velocity.z], dtype=np.float64
        )
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
                # Approaching hazard: reduce speed and increase safety margin.
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
