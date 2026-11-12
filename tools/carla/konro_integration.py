"""
Konro integration module for CARLA + CoPerception integration (TASK-03, Phase 3).

This module wraps the Konro resource manager HTTP API (already implemented in
tools/det/konro_bridge.py) to register a CARLA vehicle process with Konro,
send proxy quality feedback, and read allocated Processing Units (PUs) from
Linux cgroups/cpuset.

Responsibilities:
    1. Register the current Python process with Konro (sendAddMessage equivalent).
    2. Send feedback values [0-200] to Konro based on perception quality proxy.
    3. Read the number of PUs currently allocated to this process via cgroups.
    4. Provide fail-safe behavior if Konro is unreachable (keep current PUs).

Design notes:
    - Uses the existing konro_bridge.py HTTP client (no C++ bindings needed).
    - Konro server runs on localhost:8080 by default (env KONRO overrides).
    - If Konro is unreachable, the integration degrades gracefully: the process
      keeps its currently allocated PUs and does not crash.
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Any, Dict, Optional

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

    Attributes:
        konro_enabled: Whether Konro feedback is active.
        host: Konro server address (from KONRO env var or default).
        pid: Process ID of this Python process.
        registered: Whether this process has been registered with Konro.
        current_pus: Number of PUs currently allocated by Konro (read from cgroups).
        baseline_pus: Minimum PUs guaranteed for safety functions.
        feedback_count: Number of feedback messages sent to Konro.
    """

    def __init__(
        self,
        *,
        konro_enabled: bool = True,
        konro_target: float = 0.85,
        feedback_interval: int = 1,
        baseline_pus: int = 2,
        host: Optional[str] = None,
    ) -> None:
        """
        Initialize Konro integration.

        Args:
            konro_enabled: Whether to enable Konro feedback and registration.
            konro_target: Target quality for the proxy metric (default 0.85).
            feedback_interval: Send feedback every N frames (default 1).
            baseline_pus: Minimum PUs guaranteed for safety functions.
            host: Konro server address (e.g. "http://localhost:8080").
                  If None, uses KONRO environment variable or default.
        """
        self.konro_enabled = konro_enabled and KONRO_BRIDGE_AVAILABLE
        self.host = host or os.environ.get("KONRO", "http://localhost:8080")
        self.konro_target = konro_target
        self.feedback_interval = feedback_interval
        self.baseline_pus = baseline_pus
        self.pid = os.getpid()
        self.registered = False
        self.current_pus = baseline_pus
        self.feedback_count = 0
        self.frame_count = 0
        self.tracker: Optional[PerceptionProxyTracker] = None

        if self.konro_enabled:
            logger.info(
                "KonroIntegration initialized: pid=%d, host=%s, target=%.2f, "
                "baseline_pus=%d",
                self.pid,
                self.host,
                konro_target,
                baseline_pus,
            )
        else:
            logger.warning(
                "KonroIntegration running in fail-safe mode (konro_enabled=%s, "
                "bridge_available=%s). Baseline PUs=%d will be used.",
                konro_enabled,
                KONRO_BRIDGE_AVAILABLE,
                baseline_pus,
            )

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def register(self) -> str:
        """
        Register this process with Konro (sendAddMessage equivalent).

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
    # Feedback
    # ------------------------------------------------------------------

    def update(
        self,
        num_gts: int,
        num_dets: int,
        num_tp: int,
        frame_index: int,
    ) -> Dict[str, Any]:
        """
        Update perception proxy quality and send feedback to Konro.

        Computes the proxy metric (recall × FP penalty), tracks it via
        PerceptionProxyTracker, and sends feedback to Konro at the configured
        interval.

        Args:
            num_gts: Number of ground-truth objects in this frame.
            num_dets: Number of detections produced by the model.
            num_tp: Number of true positives.
            frame_index: Current frame index (for interval logic).

        Returns:
            Dictionary with proxy metrics and Konro feedback status.
        """
        self.frame_count += 1

        result: Dict[str, Any] = {
            "frame_index": frame_index,
            "frame_count": self.frame_count,
            "proxy": 0.0,
            "ema": 0.0,
            "feedback_sent": False,
            "feedback_value": 0,
            "pus": self.current_pus,
        }

        if not self.konro_enabled or self.tracker is None:
            result["pus"] = self.current_pus
            return result

        try:
            # Update tracker with this frame's results
            self.tracker.update(num_gts, num_dets, num_tp)

            # Read current PUs from cgroups
            from konro_bridge import _read_num_pus  # type: ignore
            self.current_pus = _read_num_pus()
            if self.current_pus < 0:
                self.current_pus = self.baseline_pus
            result["pus"] = self.current_pus

            # Send feedback at the configured interval
            if self.frame_count % self.feedback_interval == 0:
                proxy = self.tracker.current_ema
                result["proxy"] = proxy
                result["ema"] = proxy

                feedback = compute_feedback(proxy)
                result["feedback_value"] = feedback

                send_feedback_message(feedback)
                self.tracker._feedback_count += 1  # type: ignore
                self.feedback_count += 1
                result["feedback_sent"] = True

                logger.info(
                    "KonroIntegration: frame=%d proxy=%.3f feedback=%d pus=%d",
                    self.frame_count,
                    proxy,
                    feedback,
                    self.current_pus,
                )

            # Also read current PUs periodically
            if self.frame_count % self.feedback_interval == 0:
                result["pus"] = self.current_pus

        except Exception as exc:
            logger.warning("KonroIntegration: update failed: %s", exc)

        return result

    # ------------------------------------------------------------------
    # PU reading
    # ------------------------------------------------------------------

    def read_allocated_pus(self) -> int:
        """
        Read the number of PUs currently allocated to this process via cgroups.

        Returns:
            Number of PUs, or baseline_pus if reading fails.
        """
        try:
            from konro_bridge import _read_num_pus  # type: ignore
            pus = _read_num_pus()
            if pus > 0:
                self.current_pus = pus
        except Exception:
            pass
        return self.current_pus

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """Stop Konro feedback and clean up."""
        if self.tracker is not None:
            self.tracker.shutdown()
        logger.info(
            "KonroIntegration: shutdown pid=%d, feedback_sent=%d, final_pus=%d",
            self.pid,
            self.feedback_count,
            self.current_pus,
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
