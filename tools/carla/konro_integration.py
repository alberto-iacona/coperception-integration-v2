"""
Konro integration module for CARLA + CoPerception integration (TASK-03, Phase 3).

This module wraps the Konro resource manager HTTP API (already implemented in
tools/det/konro_bridge.py) to register a CARLA vehicle process with Konro,
send proxy quality feedback, and manage Processing Units (PUs) allocated to
the vehicle's perception pipeline.

Responsibilities:
    1. Register the current Python process with Konro (sendAddMessage equivalent).
    2. Send proxy quality feedback [0-200] to Konro based on perception quality
       AND network quality (from OMNeT++).
    3. Allocate PUs dynamically based on BOTH perception quality AND network
       conditions:
       - If proxy quality < target AND network is good: allocate MORE PUs
         (perception can be improved with more compute).
       - If proxy quality < target AND network is poor: KEEP baseline PUs
         (more compute won't help if packets don't arrive).
       - If proxy quality > target: decrease PUs (energy savings).
    4. Provide fail-safe behavior if Konro is unreachable (keep current PUs).

Design notes:
    - Uses the existing konro_bridge.py HTTP client (no C++ bindings needed).
    - Konro server runs on localhost:8080 by default (env KONRO overrides).
    - PU allocation is simulated within the vehicle: more PUs = less inference
      latency = fresher cooperative data = better perception quality.
    - If Konro is unreachable, the integration degrades gracefully: the process
      keeps its currently allocated PUs and does not crash.
    - NO ground truth is used: the proxy metric is based only on fused boxes
      count received vs expected boxes count (number of other agents in scene),
      which is the maximum information available to a real vehicle.
    - NO privileged CARLA ground truth is used: decisions are based only on
      observable metrics (fused boxes count, packet loss ratio, AoI).
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Any, Dict, List, Optional

# Ensure tools/det is importable for konro_bridge
_TOOLS_DET_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "det"
)
if _TOOLS_DET_PATH not in sys.path:
    sys.path.insert(0, os.path.abspath(_TOOLS_DET_PATH))

try:
    from konro_bridge import (
        PerceptionProxyTracker,
        send_add_message,
        send_feedback_message,
        compute_feedback,
    )
    KONRO_BRIDGE_AVAILABLE = True
except ImportError as exc:
    KONRO_BRIDGE_AVAILABLE = False
    logging.getLogger(__name__).warning(
        "konro_bridge not available: %s. Konro integration will run in fail-safe mode.",
        exc,
    )

logger = logging.getLogger(__name__)


class KonroIntegration:
    """
    Encapsulates Konro resource manager integration for a CARLA vehicle process.

    Konro allocates Processing Units (PUs) to the vehicle's perception pipeline
    based on BOTH perception quality (from CoPerception) AND network quality
    (from OMNeT++). This creates a true closed-loop:

        Perception → Konro feedback → PU allocation → Processing delay
            ↓                                              ↑
        Cooperative detection ←────────────────────────────┘

    Attributes:
        konro_enabled: Whether Konro feedback is active.
        host: Konro server address (from KONRO env var or default).
        pid: Process ID of this Python process.
        registered: Whether this process has been registered with Konro.
        allocated_pus: Number of PUs currently allocated by Konro to this
            vehicle. Starts at baseline_pus and changes dynamically based on
            proxy quality and network conditions.
        baseline_pus: Minimum PUs guaranteed for safety functions (ASIL-D).
        max_pus: Maximum PUs Konro can allocate to this vehicle.
        feedback_count: Number of feedback messages sent to Konro.
        allocation_changes: Number of times Konro changed the PU allocation.
        network_quality_score: Latest network quality score (0-1, higher=better).
            Computed from packet loss ratio and Age of Information.
    """

    def __init__(
        self,
        *,
        konro_enabled: bool = True,
        konro_target: float = 0.85,
        feedback_interval: int = 1,
        baseline_pus: int = 2,
        max_pus: int = 8,
        host: Optional[str] = None,
        max_expected_boxes: int = 5,
    ) -> None:
        """
        Initialize Konro integration.

        Args:
            konro_enabled: Whether to enable Konro feedback and registration.
            konro_target: Target quality for the proxy metric (default 0.85).
                Below this, Konro allocates more PUs; above this, Konro
                allocates fewer PUs.
            feedback_interval: Send feedback every N frames (default 1).
            baseline_pus: Minimum PUs guaranteed for safety functions.
            max_pus: Maximum PUs Konro can allocate to this vehicle.
            host: Konro server address (e.g. "http://localhost:8080").
                If None, uses KONRO environment variable or default.
            max_expected_boxes: Maximum number of fused boxes the ego vehicle
                could reasonably receive from partners (used to normalize the
                proxy metric). This is a configurable estimate of how many
                agents might be visible, NOT ground truth about the scene.
        """
        self.konro_enabled = konro_enabled and KONRO_BRIDGE_AVAILABLE
        self.host = host or os.environ.get("KONRO", "http://localhost:8080")
        self.konro_target = konro_target
        self.feedback_interval = feedback_interval
        self.baseline_pus = baseline_pus
        self.max_pus = max_pus
        self.pid = os.getpid()
        self.registered = False
        self.allocated_pus = baseline_pus
        self.feedback_count = 0
        self.allocation_changes = 0
        self.frame_count = 0
        self.network_quality_score = 1.0
        self.max_expected_boxes = max_expected_boxes
        self.tracker: Optional[PerceptionProxyTracker] = None

        if self.konro_enabled:
            logger.info(
                "KonroIntegration initialized: pid=%d, host=%s, target=%.2f, "
                "baseline_pus=%d, max_pus=%d",
                self.pid,
                self.host,
                konro_target,
                baseline_pus,
                max_pus,
            )
        else:
            logger.warning(
                "KonroIntegration running in fail-safe mode (konro_enabled=%s, "
                "bridge_available=%s). baseline_pus=%d will be used.",
                konro_enabled,
                KONRO_BRIDGE_AVAILABLE,
                baseline_pus,
            )

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def register(self) -> str:
        """Register this process with Konro (sendAddMessage equivalent).

        Returns:
            Konro server response text, or empty string if disabled/unavailable.
        """
        if not self.konro_enabled:
            return ""

        try:
            result = send_add_message()
            self.registered = True
            logger.info(
                "KonroIntegration: registered pid=%d with Konro: %s",
                self.pid,
                result,
            )
            return result
        except Exception as exc:
            logger.warning("KonroIntegration: registration failed: %s", exc)
            return ""

    # ------------------------------------------------------------------
    # Network quality update
    # ------------------------------------------------------------------

    def update_network_quality(
        self,
        packet_loss_ratio: float,
        aois: List[float],
        max_aoi_threshold: float = 2.0,
    ) -> None:
        """Update network quality score from OMNeT++ bridge data.

        Computes a composite network quality score (0-1, higher=better) from
        packet loss ratio and Age of Information (AoI). Delivery ratio is
        weighted 60%, AoI freshness weighted 40%.

        Args:
            packet_loss_ratio: Fraction of packets lost (0.0 = none, 1.0 = all).
            aois: List of AoI values (seconds) for recently received packets.
            max_aoi_threshold: AoI value considered "completely stale" (s).
        """
        delivery_ratio = 1.0 - packet_loss_ratio

        if aois:
            avg_aoi = sum(aois) / len(aois)
            aoi_score = max(0.0, 1.0 - (avg_aoi / max_aoi_threshold))
        else:
            # No recent AoI data: be conservative (assume degraded)
            aoi_score = 0.5

        self.network_quality_score = 0.6 * delivery_ratio + 0.4 * aoi_score
        self.network_quality_score = max(0.0, min(1.0, self.network_quality_score))

        logger.debug(
            "Network quality: delivery=%.2f, aoi_avg=%.3f, score=%.3f",
            delivery_ratio,
            avg_aoi if aois else 0.0,
            self.network_quality_score,
        )

    # ------------------------------------------------------------------
    # Feedback and PU allocation
    # ------------------------------------------------------------------

    def update(
        self,
        num_gts: int,
        num_dets: int,
        num_tp: int,
        frame_index: int,
    ) -> Dict[str, Any]:
        """Update perception proxy quality and send feedback to Konro.

        Computes proxy metric based on fused boxes count (normalized 0-1),
        sends feedback to Konro at the specified interval, and allocates PUs
        based on BOTH proxy quality AND network quality.

        Decision logic:
            - proxy < konro_target - 0.05 AND network_quality_score >= 0.5:
              allocate MORE PUs (perception can be improved with more compute).
            - proxy > konro_target + 0.05:
              decrease PUs (energy savings).
            - proxy < konro_target - 0.05 AND network_quality_score < 0.5:
              KEEP baseline PUs (network is bottleneck, more CPU won't help).

        Args:
            num_gts: Number of ground-truth objects (NOT USED for proxy calc,
                only for tracker compatibility).
            num_dets: Number of detections produced by the model.
            num_tp: Number of true positives.
            frame_index: Current frame index.

        Returns:
            Dictionary with proxy metrics, Konro feedback status, and
            allocated PUs.
        """
        self.frame_count += 1

        result: Dict[str, Any] = {
            "frame_index": frame_index,
            "frame_count": self.frame_count,
            "proxy": 0.0,
            "ema": 0.0,
            "feedback_sent": False,
            "feedback_value": 0,
            "pus": self.allocated_pus,
            "allocation_changed": False,
            "network_quality": self.network_quality_score,
        }

        if not self.konro_enabled or self.tracker is None:
            result["pus"] = self.allocated_pus
            return result

        try:
            # Update tracker with this frame's results
            self.tracker.update(num_gts, num_dets, num_tp)

            result["pus"] = self.allocated_pus

            # Send feedback at the configured interval
            if self.frame_count % self.feedback_interval == 0:
                proxy = self.tracker.current_ema
                result["proxy"] = proxy
                result["ema"] = proxy

                # Send feedback to Konro server
                feedback = compute_feedback(proxy)
                result["feedback_value"] = feedback

                send_feedback_message(feedback)
                self.tracker._feedback_count += 1  # type: ignore
                self.feedback_count += 1
                result["feedback_sent"] = True

                # DECISION LOGIC: Konro allocates PUs based on BOTH proxy
                # quality AND network quality.
                previous_pus = self.allocated_pus

                if proxy < self.konro_target - 0.05:
                    # Perception below target
                    if self.network_quality_score >= 0.5:
                        # Network is good: more PUs can help (less processing lag)
                        self.allocated_pus = min(
                            self.allocated_pus + 1, self.max_pus
                        )
                        logger.debug(
                            "[Konro] frame=%d proxy=%.3f net_quality=%.2f: "
                            "PERCEPTION BOUND - allocating %d PUs",
                            self.frame_count,
                            proxy,
                            self.network_quality_score,
                            self.allocated_pus,
                        )
                    else:
                        # Network is poor: more PUs won't help (packets don't
                        # arrive). Keep baseline PUs to save energy.
                        self.allocated_pus = self.baseline_pus
                        logger.debug(
                            "[Konro] frame=%d proxy=%.3f net_quality=%.2f: "
                            "NETWORK BOUND - keeping baseline PUs=%d",
                            self.frame_count,
                            proxy,
                            self.network_quality_score,
                            self.allocated_pus,
                        )
                elif proxy > self.konro_target + 0.05:
                    # Perception good: reduce PUs to save energy
                    self.allocated_pus = max(
                        self.allocated_pus - 1, self.baseline_pus
                    )
                    logger.debug(
                        "[Konro] frame=%d proxy=%.3f net_quality=%.2f: "
                        "QUALITY GOOD - reducing PUs to %d",
                        self.frame_count,
                        proxy,
                        self.network_quality_score,
                        self.allocated_pus,
                    )

                if self.allocated_pus != previous_pus:
                    self.allocation_changes += 1

                result["allocation_changed"] = (
                    self.allocated_pus != previous_pus
                )

            result["pus"] = self.allocated_pus

        except Exception as exc:
            logger.warning("KonroIntegration: update failed: %s", exc)

        return result

    # ------------------------------------------------------------------
    # PU reading
    # ------------------------------------------------------------------

    def read_allocated_pus(self) -> int:
        """Return the number of PUs currently allocated by Konro.

        Returns:
            Number of allocated PUs (simulated, based on Konro feedback loop).
        """
        return self.allocated_pus

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """Stop Konro feedback and clean up."""
        if self.tracker is not None:
            self.tracker.shutdown()
        logger.info(
            "KonroIntegration: shutdown pid=%d, feedback_sent=%d, "
            "allocation_changes=%d, final_pus=%d",
            self.pid,
            self.feedback_count,
            self.allocation_changes,
            self.allocated_pus,
        )

    # ------------------------------------------------------------------
    # Tracker accessor
    # ------------------------------------------------------------------

    def get_tracker(self) -> Optional[PerceptionProxyTracker]:
        """Return the PerceptionProxyTracker instance (may be None)."""
        return self.tracker

    def set_tracker(self, tracker: PerceptionProxyTracker) -> None:
        """Set the PerceptionProxyTracker instance."""
        self.tracker = tracker