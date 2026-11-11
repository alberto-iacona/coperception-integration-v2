"""
Main orchestration script for CARLA + CoPerception closed-loop simulation (TASK-01, Phase 1).

This script ties together scenario setup, sensor adapters, and cooperative agents
into a single synchronous simulation loop. It supports an arbitrary number of
vehicles and can run with cooperation enabled or disabled for ablation studies.

Usage:
    python run_closed_loop.py \
        --num-vehicles 2 \
        --map Town03 \
        --target-speed 30 \
        --ttc-threshold 3.0 \
        --cooperation \
        --duration 60 \
        --log-dir logs/carla_run

Responsibilities:
    1. Connect to CARLA server and load the map.
    2. Spawn N vehicles with LiDAR + RGB camera sensors.
    3. Create one CooperativeAgent per vehicle.
    4. Run the synchronous simulation loop:
       a. Advance world by one tick.
       b. Read sensor data and convert to PyTorch tensors.
       c. Run CoPerception feature extraction (placeholder in Phase 1).
       d. Fuse boxes and update each agent's perception.
       e. Execute control step for each agent.
    5. Log perception and control metrics to JSON.
    6. Clean up all actors on exit.

Hard constraints respected:
    - Driving decisions rely strictly on CoPerception fused bounding boxes.
    - No privileged CARLA ground-truth is used for hazard detection.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

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

from scenario_setup import ScenarioSetup
from sensor_adapter import SensorAdapter
from cooperative_agent import CooperativeAgent, register_detection, get_cooperative_boxes

logger = logging.getLogger(__name__)

# Global flag for graceful shutdown
_running = True


def signal_handler(signum: int, frame: Any) -> None:
    """Handle SIGINT/SIGTERM for graceful shutdown."""
    global _running
    logger.info("Signal %d received: shutting down gracefully...", signum)
    _running = False


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="CARLA + CoPerception closed-loop simulation (TASK-01)"
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="CARLA server IP address"
    )
    parser.add_argument(
        "--port", type=int, default=2000, help="CARLA server TCP port"
    )
    parser.add_argument(
        "--map", default="Town03", help="CARLA map to load (e.g. Town03, Town04)"
    )
    parser.add_argument(
        "--num-vehicles", type=int, default=2, help="Number of vehicles to spawn"
    )
    parser.add_argument(
        "--target-speed", type=float, default=30.0, help="Target speed in km/h"
    )
    parser.add_argument(
        "--ttc-threshold", type=float, default=3.0, help="TTC threshold in seconds"
    )
    parser.add_argument(
        "--cooperation",
        action="store_true",
        help="Enable cooperative perception fusion (default: disabled)",
    )
    parser.add_argument(
        "--no-cooperation",
        action="store_true",
        help="Explicitly disable cooperative perception (ablation baseline)",
    )
    parser.add_argument(
        "--duration", type=float, default=60.0, help="Simulation duration in seconds"
    )
    parser.add_argument(
        "--fixed-delta-seconds", type=float, default=0.05, help="Fixed time step (s)"
    )
    parser.add_argument(
        "--log-dir", default="logs/carla_run", help="Directory for output logs"
    )
    parser.add_argument(
        "--ego-id", type=int, default=None,
        help="CARLA vehicle id to treat as ego for performance analysis "
             "(default: first spawned vehicle)",
    )
    return parser.parse_args()


class ClosedLoopSimulation:
    """Encapsulates the full closed-loop simulation lifecycle."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.scenario: Optional[ScenarioSetup] = None
        self.adapter: Optional[SensorAdapter] = None
        self.agents: Dict[int, CooperativeAgent] = {}
        self.vehicle_ids: List[int] = []
        self.ego_id: Optional[int] = args.ego_id
        self.start_time: Optional[float] = None
        self.metrics: List[Dict[str, Any]] = []

        # Ensure log directory exists
        os.makedirs(args.log_dir, exist_ok=True)

    def setup(self) -> None:
        """Initialize CARLA scenario, sensors, and agents."""
        logger.info("Setting up closed-loop simulation...")
        logger.info("  Map: %s", self.args.map)
        logger.info("  Vehicles: %d", self.args.num_vehicles)
        logger.info("  Cooperation: %s", self.args.cooperation and not self.args.no_cooperation)
        logger.info("  Target speed: %.1f km/h", self.args.target_speed)
        logger.info("  TTC threshold: %.1f s", self.args.ttc_threshold)

        # 1. Scenario setup
        self.scenario = ScenarioSetup(
            host=self.args.host,
            port=self.args.port,
            map_name=self.args.map,
            fixed_delta_seconds=self.args.fixed_delta_seconds,
            synchronous_mode=True,
        )

        # 2. Spawn vehicles
        role_names = [f"vehicle_{i}" for i in range(self.args.num_vehicles)]
        vehicles = self.scenario.spawn_vehicles(
            num_vehicles=self.args.num_vehicles,
            role_names=role_names,
        )
        self.vehicle_ids = [v.id for v in vehicles]

        # Determine ego vehicle for analysis
        if self.ego_id is None and self.vehicle_ids:
            self.ego_id = self.vehicle_ids[0]
        logger.info("Ego vehicle for analysis: %d", self.ego_id)

        # 3. Attach sensors to each vehicle
        for vehicle in vehicles:
            self.scenario.attach_sensors(
                vehicle,
                enable_lidar=True,
                enable_camera=True,
            )

        # 4. Create sensor adapter
        self.adapter = SensorAdapter()

        # 5. Create one CooperativeAgent per vehicle
        cooperation_enabled = self.args.cooperation and not self.args.no_cooperation
        partner_ids = [
            vid for vid in self.vehicle_ids
        ] if cooperation_enabled else []

        for vehicle in vehicles:
            agent = CooperativeAgent(
                vehicle=vehicle,
                target_speed=self.args.target_speed,
                coop_partner_ids=partner_ids if cooperation_enabled else [],
                ttc_threshold=self.args.ttc_threshold,
                enable_cooperation=cooperation_enabled,
            )
            self.agents[vehicle.id] = agent
            logger.info(
                "Created CooperativeAgent for vehicle %d (cooperation=%s)",
                vehicle.id,
                cooperation_enabled,
            )

        self.start_time = time.time()
        logger.info("Setup complete. Simulation started.")

    def run(self) -> None:
        """Run the main synchronous simulation loop."""
        if self.scenario is None or self.adapter is None:
            raise RuntimeError("Simulation not set up. Call setup() first.")

        assert self.scenario.world is not None
        world = self.scenario.world

        logger.info("Starting simulation loop (duration=%.1fs)...", self.args.duration)

        while _running:
            elapsed = time.time() - self.start_time
            if elapsed > self.args.duration:
                logger.info("Duration limit reached (%.1fs). Stopping.", elapsed)
                break

            # Advance simulation by one tick
            world.tick()
            sim_timestamp = world.get_snapshot().timestamp.elapsed_seconds

            # Extract sensor tensors for all vehicles
            all_tensors = self.adapter.extract_all_vehicles(
                self.scenario, vehicle_ids=self.vehicle_ids
            )

            # For each vehicle: run perception + control
            for vid in self.vehicle_ids:
                agent = self.agents[vid]
                tensors = all_tensors.get(vid, {"lidar": None, "camera": None})

                # ------------------------------------------------------
                # PHASE 1 PLACEHOLDER: CoPerception feature extraction
                # ------------------------------------------------------
                # In this phase, we simulate CoPerception output by generating
                # synthetic bounding boxes from the sensor tensors.
                # In Phase 1.2+, this will call the actual CoPerception model.
                local_boxes = self._simulate_coperception_output(
                    tensors, vid, sim_timestamp
                )

                # Update agent perception (fuses local + cooperative boxes)
                agent.update_perception(local_boxes, sim_timestamp)

                # Execute control step
                control = agent.run_step()
                vehicle = self.scenario.vehicles[self.vehicle_ids.index(vid)]
                vehicle.apply_control(control)

                # Log metrics for the ego vehicle
                if vid == self.ego_id:
                    status = agent.get_perception_status()
                    self.metrics.append({
                        "timestamp": sim_timestamp,
                        "wall_clock": time.time(),
                        "vehicle_id": vid,
                        "perception_available": status["available"],
                        "consecutive_missing_frames": status["consecutive_missing_frames"],
                        "num_fused_boxes": status["num_fused_boxes"],
                        "control_throttle": control.throttle,
                        "control_brake": control.brake,
                        "control_steer": control.steer,
                        "cooperation_enabled": self.args.cooperation and not self.args.no_cooperation,
                    })

        logger.info("Simulation loop ended. Total frames logged: %d", len(self.metrics))

    def _simulate_coperception_output(
        self,
        tensors: Dict[str, Any],
        vehicle_id: int,
        timestamp: float,
    ) -> np.ndarray:
        """Placeholder for CoPerception feature extraction (Phase 1).

        In the final implementation, this method will call the CoPerception
        model (e.g. DiscoNet, V2VNet) on the LiDAR/camera tensors to produce
        fused 3D bounding boxes. For TASK-01 validation, we return a synthetic
        box representing a generic obstacle ahead of the vehicle.

        Returns:
            Numpy array [N, 7] of [x, y, z, dx, dy, dz, yaw] in world coordinates.
        """
        # TODO: Replace with actual CoPerception inference in Phase 1.2
        # For now, return empty boxes to test the "no perception" failure mode,
        # or a synthetic box ahead if we want to test basic obstacle avoidance.
        vehicle = self.scenario.vehicles[self.vehicle_ids.index(vehicle_id)]
        location = vehicle.get_location()
        # Synthetic box 10 meters ahead of the vehicle
        forward = vehicle.get_transform().get_forward_vector()
        box_x = location.x + forward.x * 10.0
        box_y = location.y + forward.y * 10.0
        box_z = location.z

        return np.array([
            [box_x, box_y, box_z, 2.0, 2.0, 2.0, 0.0]
        ], dtype=np.float32)

    def save_metrics(self) -> str:
        """Save collected metrics to a JSON file.

        Returns:
            Path to the saved JSON file.
        """
        timestamp_str = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        filename = f"carlaloop_{timestamp_str}.json"
        filepath = os.path.join(self.args.log_dir, filename)

        output = {
            "metadata": {
                "started_at_utc": datetime.now(timezone.utc).isoformat(),
                "map": self.args.map,
                "num_vehicles": self.args.num_vehicles,
                "cooperation_enabled": self.args.cooperation and not self.args.no_cooperation,
                "target_speed_kmh": self.args.target_speed,
                "ttc_threshold_s": self.args.ttc_threshold,
                "ego_vehicle_id": self.ego_id,
            },
            "metrics": self.metrics,
        }

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(output, f, indent=2)

        logger.info("Metrics saved to %s", filepath)
        return filepath

    def cleanup(self) -> None:
        """Clean up all actors and save metrics."""
        logger.info("Cleaning up simulation...")
        self.save_metrics()
        if self.scenario is not None:
            self.scenario.cleanup()
        logger.info("Cleanup complete.")


def main() -> None:
    """Entry point for the closed-loop simulation."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    args = parse_args()

    # Determine cooperation flag
    if args.cooperation and args.no_cooperation:
        logger.error("--cooperation and --no-cooperation are mutually exclusive")
        sys.exit(1)
    if not args.cooperation and not args.no_cooperation:
        # Default: cooperation disabled for baseline
        args.cooperation = False

    # Install signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    sim = ClosedLoopSimulation(args)

    try:
        sim.setup()
        sim.run()
    except Exception:
        logger.exception("Simulation failed with an unhandled exception")
        raise
    finally:
        sim.cleanup()


if __name__ == "__main__":
    main()
