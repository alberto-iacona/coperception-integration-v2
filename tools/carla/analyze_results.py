"""
Results analysis module for CARLA + CoPerception + OMNeT++ + Konro (TASK-04, Phase 4).

This module loads JSON metrics files produced by run_closed_loop.py, computes
KPIs across experimental conditions, and generates comparative charts for the
thesis evaluation campaign.

Responsibilities:
    1. Load one or more JSON metrics files from logs/carla_run/
    2. Compute aggregated KPIs per condition (perception, network, resource,
       energy, safety, comfort)
    3. Generate comparative charts (Pareto analysis, condition comparison)
    4. Save aggregated results and charts to output directory

Design notes:
    - Operates on JSON files produced by run_closed_loop.py
    - Groups metrics by condition (cooperation_enabled, konro_enabled, map,
      num_vehicles)
    - All charts saved as PNG files for thesis inclusion
"""

from __future__ import annotations

import glob
import json
import logging
import os
from typing import Any, Dict, List, Optional

import matplotlib  # type: ignore

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # type: ignore

logger = logging.getLogger(__name__)


class ResultsAnalyzer:
    """
    Encapsulates analysis and visualization of experiment results.

    Attributes:
        log_dir: Directory containing JSON metrics files.
        output_dir: Directory for saving analysis outputs.
        records: List of metric records loaded from JSON files.
    """

    def __init__(
        self,
        log_dir: str = "logs/carla_run",
        output_dir: str = "logs/carla_run/analysis",
    ) -> None:
        self.log_dir = log_dir
        self.output_dir = output_dir
        self.records: List[Dict[str, Any]] = []
        os.makedirs(output_dir, exist_ok=True)

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def load_files(self, pattern: str = "carlaloop_*.json") -> int:
        """
        Load all JSON metrics files matching the pattern from log_dir.

        Args:
            pattern: Glob pattern for JSON files.

        Returns:
            Number of records loaded.
        """
        search_path = os.path.join(self.log_dir, pattern)
        files = glob.glob(search_path)
        logger.info("Found %d metric files matching %s", len(files), search_path)

        count = 0
        for filepath in files:
            try:
                with open(filepath, "r", encoding="utf-8") as f:
                    data = json.load(f)
                metadata = data.get("metadata", {})
                metrics = data.get("metrics", [])
                for record in metrics:
                    # Attach metadata to each record for grouping
                    record["_metadata"] = {
                        "map": metadata.get("map"),
                        "num_vehicles": metadata.get("num_vehicles"),
                        "cooperation_enabled": metadata.get(
                            "cooperation_enabled"
                        ),
                        "cooperation": metadata.get("cooperation_enabled"),
                        "konro_enabled": metadata.get("konro_enabled"),
                        "target_speed_kmh": metadata.get("target_speed_kmh"),
                        "ttc_threshold_s": metadata.get("ttc_threshold_s"),
                        "ego_vehicle_id": metadata.get("ego_vehicle_id"),
                        "started_at_utc": metadata.get("started_at_utc"),
                    }
                    self.records.append(record)
                    count += 1
            except (OSError, json.JSONDecodeError) as exc:
                logger.warning("Failed to load %s: %s", filepath, exc)

        logger.info("Loaded %d metric records from %d files", count, len(files))
        return count

    # ------------------------------------------------------------------
    # Grouping
    # ------------------------------------------------------------------

    def group_by_condition(
        self,
    ) -> Dict[str, List[Dict[str, Any]]]:
        """
        Group records by experimental condition.

        Condition key format: "map|num_vehicles|cooperation|konro"

        Returns:
            Dictionary mapping condition key to list of records.
        """
        groups: Dict[str, List[Dict[str, Any]]] = {}
        for record in self.records:
            meta = record.get("_metadata", {})
            key = (
                f"{meta.get('map', 'unknown')}|"
                f"{meta.get('num_vehicles', 'unknown')}|"
                f"{meta.get('cooperation', False)}|"
                f"{meta.get('konro_enabled', False)}"
            )
            groups.setdefault(key, []).append(record)
        return groups

    # ------------------------------------------------------------------
    # KPI computation
    # ------------------------------------------------------------------

    def compute_kpis(self, records: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Compute aggregated KPIs for a list of records.

        Args:
            records: List of metric records for one condition.

        Returns:
            Dictionary with KPI averages and counts.
        """
        if not records:
            return {}

        n = len(records)

        def avg(key: str, default: float = 0.0) -> float:
            vals = [r.get(key, default) for r in records if key in r]
            return sum(vals) / len(vals) if vals else default

        def count_nonzero(key: str) -> int:
            return sum(1 for r in records if r.get(key, 0) > 0)

        perception_available = avg("perception_available", 0.0)
        num_fused_boxes = avg("num_fused_boxes", 0.0)
        consecutive_missing = avg("consecutive_missing_frames", 0.0)

        # Network KPIs
        konro_pus = avg("konro_pus", 0.0)
        konro_enabled_count = count_nonzero("konro_enabled")

        # Energy KPIs
        p_total = avg("p_total_w", 0.0)
        p_idle = avg("p_idle_w", 0.0)
        p_core = avg("p_core_w", 0.0)
        p_npu = avg("p_npu_w", 0.0)
        p_base = avg("p_base_vehicle_w", 0.0)
        energy_per_frame = avg("energy_j", 0.0)
        cumulative_energy = records[-1].get("cumulative_energy_j", 0.0) if records else 0.0

        # Control KPIs
        throttle = avg("control_throttle", 0.0)
        brake = avg("control_brake", 0.0)
        steer = avg("control_steer", 0.0)
        emergency_brake_count = count_nonzero("control_brake")  # brake == 1.0

        # Safety proxy: TTC is not directly logged, use consecutive_missing as proxy
        # for perception degradation risk
        perception_degradation = avg("consecutive_missing_frames", 0.0)

        return {
            "n_records": n,
            "perception_available": perception_available,
            "num_fused_boxes": num_fused_boxes,
            "consecutive_missing_frames": consecutive_missing,
            "perception_degradation": perception_degradation,
            "konro_pus_avg": konro_pus,
            "konro_enabled_runs": konro_enabled_count,
            "power_total_w": p_total,
            "power_idle_w": p_idle,
            "power_core_w": p_core,
            "power_npu_w": p_npu,
            "power_base_vehicle_w": p_base,
            "energy_per_frame_j": energy_per_frame,
            "cumulative_energy_j": cumulative_energy,
            "throttle_avg": throttle,
            "brake_avg": brake,
            "steer_avg": abs(steer),
            "emergency_brake_count": emergency_brake_count,
        }

    # ------------------------------------------------------------------
    # Chart generation
    # ------------------------------------------------------------------

    def generate_comparison_charts(
        self,
        grouped: Dict[str, List[Dict[str, Any]]],
    ) -> List[str]:
        """
        Generate comparative charts across conditions.

        Args:
            grouped: Dictionary mapping condition key to records.

        Returns:
            List of saved chart file paths.
        """
        saved: List[str] = []

        conditions = list(grouped.keys())
        if not conditions:
            logger.warning("No conditions to chart")
            return saved

        # Prepare data for bar charts
        kpi_names = [
            "power_total_w",
            "cumulative_energy_j",
            "num_fused_boxes",
            "consecutive_missing_frames",
            "emergency_brake_count",
        ]
        kpi_labels = [
            "Total Power (W)",
            "Cumulative Energy (J)",
            "Fused Boxes (avg)",
            "Missing Frames (avg)",
            "Emergency Brakes (count)",
        ]

        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        axes = axes.flatten()

        for idx, (kpi, label) in enumerate(zip(kpi_names, kpi_labels)):
            if idx >= len(axes):
                break
            ax = axes[idx]
            values = []
            labels = []
            for cond in conditions:
                kpis = self.compute_kpis(grouped[cond])
                values.append(kpis.get(kpi, 0.0))
                labels.append(cond.replace("|", "\n"))
            ax.bar(range(len(conditions)), values)
            ax.set_xticks(range(len(conditions)))
            ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
            ax.set_ylabel(label)
            ax.set_title(label)
            ax.grid(axis="y", alpha=0.3)

        # Hide unused subplots
        for idx in range(len(kpi_names), len(axes)):
            axes[idx].set_visible(False)

        plt.tight_layout()
        chart_path = os.path.join(self.output_dir, "comparison_charts.png")
        plt.savefig(chart_path, dpi=150, bbox_inches="tight")
        plt.close()
        logger.info("Saved comparison charts to %s", chart_path)
        saved.append(chart_path)

        # Energy vs. Perception scatter (Pareto-like)
        fig, ax = plt.subplots(figsize=(8, 6))
        x_vals: List[float] = []
        y_vals: List[float] = []
        labels_scatter: List[str] = []
        for cond in conditions:
            kpis = self.compute_kpis(grouped[cond])
            x_vals.append(kpis.get("cumulative_energy_j", 0.0))
            y_vals.append(kpis.get("num_fused_boxes", 0.0))
            labels_scatter.append(cond.replace("|", " / "))

        ax.scatter(x_vals, y_vals, s=100, alpha=0.7)
        for i, label in enumerate(labels_scatter):
            ax.annotate(label, (x_vals[i], y_vals[i]), fontsize=8, alpha=0.7)
        ax.set_xlabel("Cumulative Energy (J)")
        ax.set_ylabel("Fused Boxes (avg)")
        ax.set_title("Energy vs. Perception Quality (Pareto)")
        ax.grid(alpha=0.3)

        pareto_path = os.path.join(self.output_dir, "pareto_energy_perception.png")
        plt.savefig(pareto_path, dpi=150, bbox_inches="tight")
        plt.close()
        logger.info("Saved Pareto chart to %s", pareto_path)
        saved.append(pareto_path)

        return saved

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------

    def generate_report(
        self,
        grouped: Dict[str, List[Dict[str, Any]]],
    ) -> Dict[str, Any]:
        """
        Generate a text/JSON report with KPIs per condition.

        Args:
            grouped: Dictionary mapping condition key to records.

        Returns:
            Dictionary with report data.
        """
        report: Dict[str, Any] = {}
        for cond in sorted(grouped.keys()):
            report[cond] = self.compute_kpis(grouped[cond])
        return report

    def save_report(self, report: Dict[str, Any], filename: str = "analysis_report.json") -> str:
        """
        Save report to JSON file.

        Args:
            report: Report dictionary.
            filename: Output filename.

        Returns:
            Path to saved file.
        """
        filepath = os.path.join(self.output_dir, filename)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        logger.info("Saved analysis report to %s", filepath)
        return filepath

    def run(self, pattern: str = "carlaloop_*.json") -> Dict[str, Any]:
        """
        Execute full analysis pipeline: load, group, compute KPIs, generate charts,
        save report.

        Args:
            pattern: Glob pattern for JSON files.

        Returns:
            Report dictionary.
        """
        self.load_files(pattern)
        grouped = self.group_by_condition()
        report = self.generate_report(grouped)
        self.generate_comparison_charts(grouped)
        self.save_report(report)
        return report


def main() -> None:
    """Entry point for results analysis."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    analyzer = ResultsAnalyzer()
    report = analyzer.run()

    print("\n=== Analysis Report ===")
    for cond, kpis in report.items():
        print(f"\nCondition: {cond}")
        for key, val in kpis.items():
            print(f"  {key}: {val}")


if __name__ == "__main__":
    main()
