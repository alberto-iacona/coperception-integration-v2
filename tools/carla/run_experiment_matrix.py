"""
Experiment matrix runner for CARLA + CoPerception + OMNeT++ + Konro (TASK-04, Phase 4).

This script automatically executes multiple simulation scenarios with different
combinations of map, vehicle count, cooperation, and Konro settings, collecting
metrics for comparative analysis.

Responsibilities:
    1. Define the experiment matrix (maps, vehicle counts, conditions).
    2. Launch run_closed_loop.py for each combination.
    3. Collect exit codes and summarize results.
    4. Optionally trigger results analysis.

Design notes:
    - Each scenario runs as a subprocess with its own log file.
    - CARLA and OMNeT++ servers must be running before launching this script.
    - Konro server must be running if --konro-enabled is used.
"""

from __future__ import annotations

import itertools
import json
import logging
import os
import subprocess
import sys
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class ExperimentMatrix:
    """
    Encapsulates the experiment matrix and batch execution logic.

    Attributes:
        script_path: Path to run_closed_loop.py.
        log_dir: Directory for per-run log files.
        results: List of experiment results.
    """

    def __init__(
        self,
        script_path: str = "run_closed_loop.py",
        log_dir: str = "logs/carla_run",
    ) -> None:
        self.script_path = script_path
        self.log_dir = log_dir
        self.results: List[Dict[str, Any]] = []
        os.makedirs(log_dir, exist_ok=True)

    # ------------------------------------------------------------------
    # Matrix definition
    # ------------------------------------------------------------------

    def default_matrix(self) -> List[Dict[str, Any]]:
        """
        Define the default experiment matrix.

        Returns:
            List of experiment parameter dictionaries.
        """
        maps = ["Town03", "Town04", "Town05"]
        vehicle_counts = [2, 4, 6]
        network_conditions = ["line_of_sight", "occluded", "degraded"]
        configurations = [
            {"cooperation": False, "konro": False, "label": "baseline_local"},
            {"cooperation": True, "konro": False, "label": "coop_no_konro"},
            {"cooperation": True, "konro": True, "label": "coop_with_konro"},
        ]

        experiments: List[Dict[str, Any]] = []
        for map_name, num_vehicles, net_cond, config in itertools.product(
            maps, vehicle_counts, network_conditions, configurations
        ):
            exp: Dict[str, Any] = {
                "map": map_name,
                "num_vehicles": num_vehicles,
                "network_condition": net_cond,
                "cooperation": config["cooperation"],
                "konro": config["konro"],
                "label": config["label"],
                "target_speed": 30.0,
                "ttc_threshold": 3.0,
                "duration": 30.0,
                "fixed_delta_seconds": 0.05,
            }
            experiments.append(exp)

        logger.info("Defined %d experiments in default matrix", len(experiments))
        return experiments

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def build_command(self, exp: Dict[str, Any]) -> List[str]:
        """
        Build the command-line invocation for a single experiment.

        Args:
            exp: Experiment parameter dictionary.

        Returns:
            List of command-line arguments.
        """
        cmd: List[str] = [
            "python",
            self.script_path,
            "--map",
            exp["map"],
            "--num-vehicles",
            str(exp["num_vehicles"]),
            "--target-speed",
            str(exp["target_speed"]),
            "--ttc-threshold",
            str(exp["ttc_threshold"]),
            "--duration",
            str(exp["duration"]),
            "--fixed-delta-seconds",
            str(exp["fixed_delta_seconds"]),
            "--log-dir",
            self.log_dir,
        ]

        # Cooperation flags
        if exp["cooperation"]:
            cmd.append("--cooperation")
        else:
            cmd.append("--no-cooperation")

        # OMNeT++ flags (always enabled when cooperation is on)
        if exp["cooperation"]:
            cmd.extend(["--omnet-host", "127.0.0.1", "--omnet-port", "5555"])

        # Konro flags
        if exp["konro"]:
            cmd.extend(
                [
                    "--konro-enabled",
                    "--konro-target",
                    "0.85",
                    "--konro-host",
                    "http://localhost:8080",
                    "--konro-port",
                    "8080",
                    "--konro-feedback-interval",
                    "1",
                    "--konro-baseline-pus",
                    "2",
                ]
            )
        else:
            cmd.append("--no-omnet")  # disable OMNeT++ bridge when no cooperation

        return cmd

    def run_single(self, exp: Dict[str, Any], index: int, total: int) -> Dict[str, Any]:
        """
        Run a single experiment.

        Args:
            exp: Experiment parameter dictionary.
            index: 1-based index of current experiment.
            total: Total number of experiments.

        Returns:
            Result dictionary with status, returncode, and parameters.
        """
        cmd = self.build_command(exp)
        cmd_str = " ".join(cmd)
        logger.info(
            "[%d/%d] Running: %s", index, total, cmd_str
        )

        log_file = os.path.join(
            self.log_dir, f"exp_{index:03d}_{exp['label']}.log"
        )

        start_time = __import__("time").time()
        try:
            with open(log_file, "w", encoding="utf-8") as lf:
                proc = subprocess.run(
                    cmd,
                    stdout=lf,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=120,  # 2 minutes per run
                )
            returncode = proc.returncode
        except subprocess.TimeoutExpired:
            logger.warning("Experiment %d timed out", index)
            returncode = -1
        except Exception as exc:
            logger.error("Experiment %d failed: %s", index, exc)
            returncode = -2

        elapsed = __import__("time").time() - start_time

        result: Dict[str, Any] = {
            "index": index,
            "label": exp["label"],
            "map": exp["map"],
            "num_vehicles": exp["num_vehicles"],
            "network_condition": exp["network_condition"],
            "cooperation": exp["cooperation"],
            "konro": exp["konro"],
            "returncode": returncode,
            "elapsed_s": round(elapsed, 2),
            "log_file": log_file,
            "success": returncode == 0,
        }
        self.results.append(result)
        return result

    def run_all(self, experiments: Optional[List[Dict[str, Any]]] = None) -> None:
        """
        Run all experiments in the matrix.

        Args:
            experiments: List of experiment parameters. If None, uses default_matrix().
        """
        if experiments is None:
            experiments = self.default_matrix()

        total = len(experiments)
        logger.info("Starting experiment matrix with %d runs", total)

        for idx, exp in enumerate(experiments, start=1):
            self.run_single(exp, idx, total)

        self.print_summary()

    def print_summary(self) -> None:
        """Print a summary of all experiment results."""
        print("\n=== Experiment Matrix Summary ===")
        total = len(self.results)
        successful = sum(1 for r in self.results if r["success"])
        failed = total - successful
        print(f"Total runs: {total}")
        print(f"Successful: {successful}")
        print(f"Failed: {failed}")

        print("\nPer-condition results:")
        for r in self.results:
            status = "OK" if r["success"] else "FAIL"
            print(
                f"  [{r['index']:03d}] {r['label']:20s} "
                f"map={r['map']:9s} nv={r['num_vehicles']} "
                f"net={r['network_condition']:12s} "
                f"konro={str(r['konro']):5s} "
                f"{status} ({r['elapsed_s']}s)"
            )

        # Save summary to JSON
        summary_path = os.path.join(self.log_dir, "experiment_matrix_summary.json")
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "total": total,
                    "successful": successful,
                    "failed": failed,
                    "results": self.results,
                },
                f,
                indent=2,
            )
        logger.info("Saved experiment summary to %s", summary_path)


def main() -> None:
    """Entry point for experiment matrix runner."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    matrix = ExperimentMatrix()
    matrix.run_all()


if __name__ == "__main__":
    main()
