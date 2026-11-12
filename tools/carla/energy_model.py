"""
Energy model module for CARLA + CoPerception integration (TASK-03, Phase 3).

This module implements the on-board vehicle energy consumption model used by
Konro for dynamic CPU resource allocation. The model follows the formulation
from Lin et al. (ASPLOS 2018) and DLR component-level analysis:

    P_compute(t) = P_idle + sum(P_core_i * U_core_i(t)) + P_NPU(t) + P_base_vehicle

Responsibilities:
    1. Model instantaneous power consumption of the on-board computer (SoC).
    2. Map Konro-allocated Processing Units (PUs) to CPU core utilization.
    3. Compute cumulative energy consumption over simulation time.
    4. Provide per-frame power breakdown for logging and Pareto analysis.

Design notes:
    - All parameters are configurable via constructor arguments.
    - Default values are parametric and derived from bibliographic research
      (see docs/thesis/ENERGY_MODEL_PARAMETERS.md).
    - P_base_vehicle represents safety-critical functions (braking, steering,
      sensors, V2X) that must never be compromised by Konro CPU throttling.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class VehicleEnergyModel:
    """
    Encapsulates the energy consumption model of an on-board automotive computer.

    Attributes:
        p_idle: Static board/memory power in Watts (typically 15%–25% of TDP).
        p_core: Power of a single CPU core at 100% utilization in Watts.
        p_npu: NPU/GPU accelerator power during active inference in Watts.
        p_base_vehicle: Power of base vehicle safety functions in Watts.
        num_cores_total: Total number of CPU cores available on the SoC.
        baseline_pus: Minimum number of PUs guaranteed for safety functions.
    """

    def __init__(
        self,
        *,
        p_idle: float = 20.0,
        p_core: float = 2.0,
        p_npu: float = 25.0,
        p_base_vehicle: float = 100.0,
        num_cores_total: int = 8,
        baseline_pus: int = 2,
    ) -> None:
        """
        Initialize the energy model with configurable parameters.

        Args:
            p_idle: Static idle power of the on-board computer in Watts.
            p_core: Power per CPU core at full utilization in Watts.
            p_npu: NPU/GPU accelerator power during active inference in Watts.
            p_base_vehicle: Base vehicle safety function power in Watts.
            num_cores_total: Total CPU cores available on the SoC.
            baseline_pus: Minimum PUs guaranteed for safety-critical functions.
        """
        self.p_idle = p_idle
        self.p_core = p_core
        self.p_npu = p_npu
        self.p_base_vehicle = p_base_vehicle
        self.num_cores_total = num_cores_total
        self.baseline_pus = baseline_pus

        # Cumulative energy tracking
        self.cumulative_energy_j: float = 0.0
        self.last_timestamp_s: Optional[float] = None

        # Per-frame power history for logging
        self.power_history: list = []

        logger.info(
            "VehicleEnergyModel initialized: p_idle=%.1fW, p_core=%.1fW/core, "
            "p_npu=%.1fW, p_base=%.1fW, cores=%d, baseline_pus=%d",
            p_idle,
            p_core,
            p_npu,
            p_base_vehicle,
            num_cores_total,
            baseline_pus,
        )

    # ------------------------------------------------------------------
    # Power computation
    # ------------------------------------------------------------------

    def compute_power(
        self,
        num_pus: int,
        cpu_utilization: float = 1.0,
        npu_active: bool = True,
    ) -> float:
        """
        Compute instantaneous power consumption in Watts.

        Args:
            num_pus: Number of Processing Units (CPU cores) currently allocated
                     by Konro. Must be >= baseline_pus.
            cpu_utilization: Average CPU utilization across allocated cores
                             in [0.0, 1.0]. Defaults to 1.0 (full load).
            npu_active: Whether the NPU/GPU accelerator is active during
                        inference. Defaults to True.

        Returns:
            Instantaneous power consumption in Watts.
        """
        # Clamp num_pus to valid range
        num_pus = max(self.baseline_pus, min(num_pus, self.num_cores_total))

        # Core power: sum over allocated cores
        core_power = self.p_core * num_pus * cpu_utilization

        # NPU power: active only if inference is running
        npu_power = self.p_npu if npu_active else 0.0

        # Total compute power
        p_compute = self.p_idle + core_power + npu_power

        # Total on-board power including base vehicle safety functions
        p_total = p_compute + self.p_base_vehicle

        logger.debug(
            "Power: idle=%.1fW cores=%.1fW npu=%.1fW base=%.1fW total=%.1fW (pus=%d)",
            self.p_idle,
            core_power,
            npu_power,
            self.p_base_vehicle,
            p_total,
            num_pus,
        )
        return p_total

    def compute_energy(self, power_w: float, duration_s: float) -> float:
        """
        Compute energy consumption in Joules for a given duration.

        Args:
            power_w: Instantaneous power in Watts.
            duration_s: Duration in seconds.

        Returns:
            Energy consumption in Joules.
        """
        energy_j = power_w * duration_s
        self.cumulative_energy_j += energy_j
        logger.debug(
            "Energy: %.3fW * %.3fs = %.3fJ (cumulative=%.3fJ)",
            power_w,
            duration_s,
            energy_j,
            self.cumulative_energy_j,
        )
        return energy_j

    def record_frame(
        self,
        timestamp_s: float,
        num_pus: int,
        cpu_utilization: float,
        npu_active: bool,
        duration_s: float,
    ) -> Dict[str, Any]:
        """
        Record one frame's power and energy consumption.

        Args:
            timestamp_s: Simulation timestamp in seconds.
            num_pus: Number of PUs allocated by Konro.
            cpu_utilization: Average CPU utilization in [0.0, 1.0].
            npu_active: Whether NPU was active during this frame.
            duration_s: Frame duration in seconds.

        Returns:
            Dictionary with power and energy breakdown for this frame.
        """
        power_w = self.compute_power(num_pus, cpu_utilization, npu_active)
        energy_j = self.compute_energy(power_w, duration_s)

        record: Dict[str, Any] = {
            "timestamp_s": timestamp_s,
            "num_pus": num_pus,
            "cpu_utilization": cpu_utilization,
            "npu_active": npu_active,
            "p_idle_w": self.p_idle,
            "p_core_w": self.p_core * num_pus * cpu_utilization,
            "p_npu_w": self.p_npu if npu_active else 0.0,
            "p_base_vehicle_w": self.p_base_vehicle,
            "p_total_w": power_w,
            "energy_j": energy_j,
            "cumulative_energy_j": self.cumulative_energy_j,
        }

        self.power_history.append(record)
        self.last_timestamp_s = timestamp_s

        logger.info(
            "Frame t=%.3fs: pus=%d, power=%.2fW, energy=%.3fJ (cum=%.3fJ)",
            timestamp_s,
            num_pus,
            power_w,
            energy_j,
            self.cumulative_energy_j,
        )
        return record

    def get_power_breakdown(self) -> Dict[str, Any]:
        """
        Return a summary of the power model configuration.

        Returns:
            Dictionary with model parameters and cumulative statistics.
        """
        return {
            "p_idle_w": self.p_idle,
            "p_core_w": self.p_core,
            "p_npu_w": self.p_npu,
            "p_base_vehicle_w": self.p_base_vehicle,
            "num_cores_total": self.num_cores_total,
            "baseline_pus": self.baseline_pus,
            "cumulative_energy_j": self.cumulative_energy_j,
            "num_frames_recorded": len(self.power_history),
        }

    def reset(self) -> None:
        """Reset cumulative energy and frame history."""
        self.cumulative_energy_j = 0.0
        self.last_timestamp_s = None
        self.power_history.clear()
        logger.info("Energy model reset.")
