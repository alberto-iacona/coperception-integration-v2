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
from cooperative_agent import CooperativeAgent
from omnet_bridge import OmnetBridge
from energy_model import VehicleEnergyModel
from konro_integration import KonroIntegration

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
    # ------------------------------------------------------------------
    # OMNeT++ bridge options
    # ------------------------------------------------------------------
    parser.add_argument(
        "--omnet-host", default="127.0.0.1", help="OMNeT++ server IP address"
    )
    parser.add_argument(
        "--omnet-port", type=int, default=5555, help="OMNeT++ server TCP port"
    )
    parser.add_argument(
        "--omnet-timeout", type=float, default=2.0, help="Socket timeout (s)"
    )
    parser.add_argument(
        "--omnet-default-delay", type=float, default=0.0,
        help="Fallback delay in seconds when the bridge is unavailable",
    )
    parser.add_argument(
        "--omnet-fail-open",
        action="store_true",
        help="Continue with default decisions when the bridge disconnects",
    )
    parser.add_argument(
        "--no-omnet",
        action="store_true",
        help="Disable OMNeT++ bridge (run without network simulation)",
    )
    # ------------------------------------------------------------------
    # Konro resource manager options
    # ------------------------------------------------------------------
    parser.add_argument(
        "--konro-enabled",
        action="store_true",
        help="Enable Konro resource manager feedback (default: disabled)",
    )
    parser.add_argument(
        "--konro-target", type=float, default=0.85,
        help="Target proxy quality for Konro feedback (default: 0.85)",
    )
    parser.add_argument(
        "--konro-host", default="http://localhost:8080",
        help="Konro server address (default: http://localhost:8080)",
    )
    parser.add_argument(
        "--konro-port", type=int, default=8080,
        help="Konro server port (default: 8080)",
    )
    parser.add_argument(
        "--konro-feedback-interval", type=int, default=1,
        help="Send feedback to Konro every N frames (default: 1)",
    )
    parser.add_argument(
        "--konro-baseline-pus", type=int, default=2,
        help="Minimum PUs guaranteed for safety functions (default: 2)",
    )
    parser.add_argument(
        "--num-pedestrians", type=int, default=4,
        help="Number of pedestrians (walkers) to spawn (default: 4)",
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
        self.energy_model: Optional[VehicleEnergyModel] = None
        self.konro: Optional[KonroIntegration] = None

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

        # 3b. Spawn pedestrians (walkers) - no sensors, detectable by vehicles
        self.scenario.spawn_pedestrians(
            num_pedestrians=self.args.num_pedestrians,
            speed=1.0,  # walking speed
        )

        # 4. Create sensor adapter
        self.adapter = SensorAdapter()

        # 5. Create OMNeT++ bridge (central bridge for all vehicles)
        omnet_enabled = self.args.cooperation and not self.args.no_cooperation and not self.args.no_omnet
        self.bridge: Optional[OmnetBridge] = None
        if omnet_enabled:
            self.bridge = OmnetBridge(
                host=self.args.omnet_host,
                port=self.args.omnet_port,
                timeout=self.args.omnet_timeout,
                default_delay=self.args.omnet_default_delay,
                fail_open=self.args.omnet_fail_open,
                enabled=True,
            )
            logger.info(
                "OMNeT++ bridge enabled: %s:%d",
                self.args.omnet_host,
                self.args.omnet_port,
            )
        else:
            logger.info("OMNeT++ bridge disabled (no network simulation)")

        # 6. Create energy model
        self.energy_model = VehicleEnergyModel(
            p_idle=20.0,
            p_core=2.0,
            p_npu=25.0,
            p_base_vehicle=100.0,
            num_cores_total=8,
            baseline_pus=self.args.konro_baseline_pus,
        )
        logger.info(
            "Energy model created: p_idle=%.1fW p_core=%.1fW p_npu=%.1fW "
            "p_base=%.1fW cores=%d baseline_pus=%d",
            self.energy_model.p_idle,
            self.energy_model.p_core,
            self.energy_model.p_npu,
            self.energy_model.p_base_vehicle,
            self.energy_model.num_cores_total,
            self.energy_model.baseline_pus,
        )

        # 7. Create Konro integration
        konro_enabled = self.args.konro_enabled and not self.args.no_omnet
        self.konro: Optional[KonroIntegration] = None
        if konro_enabled:
            self.konro = KonroIntegration(
                konro_enabled=True,
                konro_target=self.args.konro_target,
                feedback_interval=self.args.konro_feedback_interval,
                baseline_pus=self.args.konro_baseline_pus,
                host=self.args.konro_host,
            )
            self.konro.register()
            logger.info(
                "Konro integration enabled: target=%.2f baseline_pus=%d host=%s",
                self.args.konro_target,
                self.args.konro_baseline_pus,
                self.args.konro_host,
            )
        else:
            logger.info("Konro integration disabled")

        # 8. Create one CooperativeAgent per vehicle
        cooperation_enabled = self.args.cooperation and not self.args.no_cooperation
        partner_ids = [
            vid for vid in self.vehicle_ids
        ] if cooperation_enabled else []

        for vehicle in vehicles:
            agent = CooperativeAgent(
                vehicle=vehicle,
                bridge=self.bridge,
                energy_model=self.energy_model,
                konro=self.konro,
                target_speed=self.args.target_speed,
                coop_partner_ids=partner_ids if cooperation_enabled else [],
                ttc_threshold=self.args.ttc_threshold,
                enable_cooperation=cooperation_enabled and omnet_enabled,
            )
            self.agents[vehicle.id] = agent
            logger.info(
                "Created CooperativeAgent for vehicle %d (cooperation=%s, omnet=%s, konro=%s)",
                vehicle.id,
                cooperation_enabled,
                omnet_enabled,
                konro_enabled,
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

            # Update OMNeT++ bridge state (read pending network responses)
            if self.bridge is not None:
                self.bridge.update_state()

            # Extract sensor tensors for all vehicles
            all_tensors = self.adapter.extract_all_vehicles(
                self.scenario, vehicle_ids=self.vehicle_ids
            )

            # For each vehicle: update position in OMNeT++, run perception + control
            for vid in self.vehicle_ids:
                agent = self.agents[vid]
                vehicle = self.scenario.vehicles[self.vehicle_ids.index(vid)]
                tensors = all_tensors.get(vid, {"lidar": None, "camera": None})

                # Send vehicle position to OMNeT++ for network simulation
                if self.bridge is not None:
                    location = vehicle.get_location()
                    self.bridge.update_position(
                        vehicle_id=vid,
                        x=location.x,
                        y=location.y,
                        z=location.z,
                    )

                # ------------------------------------------------------
                # PHASE 1 PLACEHOLDER: CoPerception feature extraction
                # ------------------------------------------------------
                # In this phase, we simulate CoPerception output by generating
                # synthetic bounding boxes from the sensor tensors.
                # In Phase 1.2+, this will call the actual CoPerception model.
                local_boxes = self._simulate_coperception_output(
                    tensors, vid, sim_timestamp
                )

                # Update agent perception (fuses local + cooperative boxes through OMNeT++)
                agent.update_perception(local_boxes, sim_timestamp)

                # Read allocated PUs from Konro and update agent
                konro_pus = self.args.konro_baseline_pus
                if self.konro is not None:
                    self.konro.read_allocated_pus()
                    konro_pus = self.konro.current_pus
                agent.set_konro_pus(konro_pus)

                # Execute control step
                control = agent.run_step()
                vehicle.apply_control(control)

                # Update Konro with proxy quality metric (use fused boxes count as proxy)
                if self.konro is not None:
                    # Proxy metric: based on fused boxes count (normalized 0-1)
                    num_boxes = status["num_fused_boxes"] if (status := agent.get_perception_status()) else 0
                    proxy = min(1.0, num_boxes / 5.0)  # 5 boxes = full quality proxy
                    self.konro.update(
                        num_gts=max(1, num_boxes),
                        num_dets=max(1, num_boxes),
                        num_tp=num_boxes,
                        frame_index=agent.get_perception_status()["consecutive_missing_frames"],
                    )

                # Record energy for this frame
                energy_record = None
                if self.energy_model is not None:
                    cpu_util = min(1.0, konro_pus / float(self.energy_model.num_cores_total))
                    energy_record = self.energy_model.record_frame(
                        timestamp_s=sim_timestamp,
                        num_pus=konro_pus,
                        cpu_utilization=cpu_util,
                        npu_active=True,
                        duration_s=self.args.fixed_delta_seconds,
                    )

                # Log metrics for the ego vehicle
                if vid == self.ego_id:
                    status = agent.get_perception_status()
                    metric_entry = {
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
                        "omnet_enabled": self.bridge is not None and self.bridge.enabled,
                        "konro_enabled": self.konro is not None and self.konro.konro_enabled,
                        "konro_pus": konro_pus,
                    }
                    if energy_record is not None:
                        metric_entry.update({
                            "p_total_w": energy_record["p_total_w"],
                            "p_idle_w": energy_record["p_idle_w"],
                            "p_core_w": energy_record["p_core_w"],
                            "p_npu_w": energy_record["p_npu_w"],
                            "p_base_vehicle_w": energy_record["p_base_vehicle_w"],
                            "energy_j": energy_record["energy_j"],
                            "cumulative_energy_j": energy_record["cumulative_energy_j"],
                        })
                    self.metrics.append(metric_entry)

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
        """Clean up all actors, save metrics, close OMNeT++ bridge and Konro."""
        logger.info("Cleaning up simulation...")
        self.save_metrics()
        if self.scenario is not None:
            self.scenario.cleanup()
        if self.bridge is not None:
            self.bridge.close()
        if self.konro is not None:
            self.konro.shutdown()
        if self.energy_model is not None:
            logger.info(
                "Energy model summary: %s",
                self.energy_model.get_power_breakdown(),
            )
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
