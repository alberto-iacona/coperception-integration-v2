"""
OMNeT++ network bridge for CARLA + CoPerception integration (TASK-02, Phase 2).

This module provides an asynchronous TCP client that communicates with the
OMNeT++ NetworkManager (IEEE 802.11p V2X network simulator).

Responsibilities:
    1. Connect to OMNeT++ server on configurable host/port (default 127.0.0.1:5555).
    2. Send 'move' commands to update vehicle positions in the network simulator.
    3. Send 'send' commands to request feature map transmissions between vehicles.
    4. Receive 'received' notifications with delay and deliver status.
    5. Track packet transmission statistics (sent, delivered, lost) for network
       quality estimation.
    6. Track Age of Information (AoI) for received packets.
    7. Store published bounding boxes per vehicle and provide them to partners
       through the simulated network (deliver/delay decided by OMNeT++).
    8. Provide network decisions (delivered/lost, delay) to CooperativeAgent.

Design notes:
    - Single TCP connection shared by all vehicles (central bridge pattern).
    - Socket is non-blocking for asynchronous operation.
    - Pending decisions are tracked per message ID and resolved when
      OMNeT++ sends the corresponding 'received' notification.
    - No ground-truth data is used: only network-simulated delivery outcomes.
"""

from __future__ import annotations

import json
import logging
import socket
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class OmnetBridge:
    """
    Asynchronous TCP client for OMNeT++ NetworkManager communication.

    Attributes:
        host: OMNeT++ server IP address.
        port: OMNeT++ server TCP port.
        timeout: Socket connection timeout in seconds.
        default_delay: Fallback delay in seconds when bridge is unavailable.
        fail_open: If True, continue with default decisions when disconnected.
        enabled: Whether the bridge is active.
    """

    def __init__(
        self,
        *,
        host: str,
        port: int,
        timeout: float = 2.0,
        default_delay: float = 0.0,
        fail_open: bool = False,
        enabled: bool = True,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.default_delay = max(0.0, default_delay)
        self.fail_open = fail_open
        self.enabled = enabled

        self._sock: Optional[socket.socket] = None
        self._buffer = ""
        self._connected = False

        # Cache of latest known delays per pair: "sender->receiver" -> delay_s
        self.latest_delays: Dict[str, float] = {}
        # Pending losses per pair: "sender->receiver" -> count of packets to drop
        self._pending_losses: Dict[str, int] = {}
        # Sequence counter per pair for unique message IDs
        self._next_seq: Dict[str, int] = {}

        # Registry of latest boxes published by each vehicle (world coordinates)
        # Structure: { vehicle_id: {"boxes": np.ndarray[N, 7], "timestamp": float} }
        self._published_boxes: Dict[int, Dict[str, Any]] = {}

        # Network quality tracking
        # Total packets sent via transmit()
        self._packets_sent: int = 0
        # Packets delivered (received 'received' notification from OMNeT++)
        self._packets_delivered: int = 0
        # Packets lost (sent but no ACK received within timeout, or explicit drop)
        self._packets_lost: int = 0
        # Timestamp when each packet was sent: msg_id -> send_time
        self._send_timestamps: Dict[str, float] = {}
        # AoI values for received packets: list of (age_at_receipt) values
        self._aoi_history: List[float] = []
        # Max AoI history length to keep in memory
        self._max_aoi_history: int = 100

        if self.enabled:
            self._connect()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        """Establish TCP connection to OMNeT++ server."""
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            sock.setblocking(False)
            self._sock = sock
            self._buffer = ""
            self._connected = True
            logger.info("OmnetBridge connected to %s:%d", self.host, self.port)
        except OSError as exc:
            logger.warning(
                "[OmnetBridge] unable to connect to %s:%d (%s)", self.host, self.port, exc
            )
            if not self.fail_open:
                raise
            self.enabled = False
            self._connected = False

    def close(self) -> None:
        """Close the TCP connection."""
        self._connected = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        logger.info("OmnetBridge connection closed.")

    # ------------------------------------------------------------------
    # Box publishing / retrieval (cooperative data through simulated network)
    # ------------------------------------------------------------------

    def publish_boxes(
        self,
        vehicle_id: int,
        boxes: np.ndarray,
        timestamp: float,
    ) -> None:
        """Publish fused bounding boxes for a vehicle.

        Makes the boxes available to partner vehicles through the simulated
        network. Partners retrieve them via get_partner_boxes(), which asks
        OMNeT++ whether the transmission would be delivered and with what delay.

        Args:
            vehicle_id: CARLA vehicle ID publishing the boxes.
            boxes: Numpy array [N, 7] with [x, y, z, dx, dy, dz, yaw] in world
                   coordinates.
            timestamp: Simulation timestamp of the detection.
        """
        self._published_boxes[vehicle_id] = {
            "boxes": boxes,
            "timestamp": timestamp,
        }
        logger.debug(
            "Published %d boxes for vehicle %d at t=%.3f",
            len(boxes) if boxes is not None else 0,
            vehicle_id,
            timestamp,
        )

    def get_partner_boxes(
        self,
        receiver_id: int,
        partner_id: int,
    ) -> Tuple[Optional[np.ndarray], float, bool]:
        """Retrieve partner vehicle boxes through simulated OMNeT++ transmission.

        Sends a 'send' command to OMNeT++ requesting transmission from partner
        to receiver. OMNeT++ decides deliver/delay based on simulated radio
        conditions (distance, interference, etc.).

        Args:
            receiver_id: CARLA vehicle ID requesting the boxes.
            partner_id: CARLA vehicle ID whose boxes are requested.

        Returns:
            Tuple of (boxes, delay_s, delivered).
            - boxes: Partner's fused boxes if delivered, None otherwise.
            - delay_s: Network delay in seconds (0.0 if not delivered).
            - delivered: True if OMNeT++ decided the packet is delivered.
        """
        partner_data = self._published_boxes.get(partner_id)
        if partner_data is None:
            return None, 0.0, False

        # Request transmission through OMNeT++
        decision = self.transmit(
            sender=partner_id,
            receiver=receiver_id,
            size_bytes=1000,  # Feature map size estimate in bytes
        )

        if not decision["deliver"]:
            return None, 0.0, False

        delay_s = decision["delay_s"]
        boxes = partner_data["boxes"]
        logger.debug(
            "Partner boxes delivered: vehicle %d <- %d (delay=%.3fs, %d boxes)",
            receiver_id,
            partner_id,
            delay_s,
            len(boxes) if boxes is not None else 0,
        )
        return boxes, delay_s, True

    # ------------------------------------------------------------------
    # Sending commands to OMNeT++
    # ------------------------------------------------------------------

    def update_position(
        self,
        vehicle_id: int,
        x: float,
        y: float,
        z: float,
    ) -> None:
        """Send a 'move' command to update vehicle position in OMNeT++.

        Args:
            vehicle_id: CARLA vehicle ID.
            x: X coordinate in meters.
            y: Y coordinate in meters.
            z: Z coordinate in meters.
        """
        if not self.enabled or not self._sock:
            return

        payload = {
            "type": "move",
            "id": str(vehicle_id),
            "x": str(x),
            "y": str(y),
            "z": str(z),
        }
        self._send_json(payload)

    def transmit(
        self,
        *,
        sender: int,
        receiver: int,
        size_bytes: int,
        msg_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Request a feature map transmission through OMNeT++.

        Sends a 'send' command and returns a network decision based on
        previously observed channel conditions for this pair.

        Args:
            sender: Source vehicle ID.
            receiver: Destination vehicle ID.
            size_bytes: Payload size in bytes.
            msg_id: Optional unique message ID. If None, a sequence-based
                    ID is generated.

        Returns:
            Dictionary with keys "deliver" (bool) and "delay_s" (float).
        """
        if not self.enabled or not self._sock:
            return {"deliver": True, "delay_s": self.default_delay}

        pair_key = f"{sender}->{receiver}"
        seq = self._next_seq.get(pair_key, 0)
        self._next_seq[pair_key] = seq + 1

        if msg_id is None:
            msg_id = f"{pair_key}#{seq}"

        payload = {
            "type": "send",
            "src": str(sender),
            "dst": str(receiver),
            "size": str(size_bytes),
            "id": msg_id,
        }

        try:
            self._send_json(payload)
        except OSError as exc:
            logger.warning(
                "[OmnetBridge] send error (%s); fail_open=%s", exc, self.fail_open
            )
            if self.fail_open:
                self.enabled = False
                return {"deliver": True, "delay_s": self.default_delay}
            return {"deliver": False, "delay_s": 0.0}

        # Record packet sent for network quality tracking
        self._packets_sent += 1
        self._send_timestamps[msg_id] = time.time()

        # Channel delay: last known for this pair
        delay = max(0.0, self.latest_delays.get(pair_key, self.default_delay))

        # Inject pending loss if previously observed
        pending = self._pending_losses.get(pair_key, 0)
        if pending > 0:
            self._pending_losses[pair_key] = pending - 1
            self._packets_lost += 1
            return {"deliver": False, "delay_s": delay}

        return {"deliver": True, "delay_s": delay}

    # ------------------------------------------------------------------
    # Receiving responses from OMNeT++
    # ------------------------------------------------------------------

    def update_state(self) -> None:
        """Read pending responses from OMNeT++ and update channel state.

        Processes all complete lines in the socket buffer. Each line is
        expected to be a JSON notification of type "received".
        """
        if not self.enabled or not self._sock:
            return

        try:
            while True:
                try:
                    chunk = self._sock.recv(4096)
                    if not chunk:
                        break  # Connection closed
                    self._buffer += chunk.decode("utf-8")
                except BlockingIOError:
                    break
                except ConnectionResetError:
                    self._close()
                    return
        except Exception as e:
            logger.warning("[OmnetBridge] Error reading socket: %s", e)
            return

        # Process complete lines
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            try:
                msg = json.loads(line)
                if msg.get("type") == "received":
                    msg_id = msg.get("id", "")
                    delay = float(msg.get("delay", 0.0))
                    deliver = bool(msg.get("deliver", True))

                    if not msg_id:
                        continue

                    pair_key, seq = self._parse_msg_id(msg_id)
                    if not pair_key:
                        continue

                    # Always update the known delay for this pair
                    self.latest_delays[pair_key] = delay

                    # Track packet delivery statistics
                    if not deliver:
                        # Explicit delivery failure
                        self._packets_lost += 1
                    else:
                        self._packets_delivered += 1
                        # Calculate AoI: current time - send time of this packet
                        send_time = self._send_timestamps.pop(msg_id, None)
                        if send_time is not None:
                            aoi = time.time() - send_time
                            self._aoi_history.append(aoi)
                            # Keep history bounded
                            if len(self._aoi_history) > self._max_aoi_history:
                                self._aoi_history = self._aoi_history[-self._max_aoi_history:]

                    # Track explicit delivery failure as pending loss
                    if not deliver:
                        self._pending_losses[pair_key] = (
                            self._pending_losses.get(pair_key, 0) + 1
                        )

            except json.JSONDecodeError:
                pass

    # ------------------------------------------------------------------
    # Network quality metrics
    # ------------------------------------------------------------------

    def get_packet_loss_ratio(self) -> float:
        """Return packet loss ratio (0.0 = no loss, 1.0 = all lost)."""
        if self._packets_sent == 0:
            return 0.0
        return self._packets_lost / self._packets_sent

    def get_recent_aois(self, max_age_seconds: float = 5.0) -> List[float]:
        """Return list of Age of Information values for recent packets.

        Args:
            max_age_seconds: Maximum age in seconds to consider an AoI value
                             "recent". Older values are discarded.

        Returns:
            List of AoI values in seconds for recently received packets.
        """
        current_time = time.time()
        # Filter to recent values only
        recent = [a for a in self._aoi_history if (current_time - a) <= max_age_seconds]
        return recent

    def get_average_delay(self) -> float:
        """Return average delay across all pairs (seconds)."""
        if not self.latest_delays:
            return 0.0
        return sum(self.latest_delays.values()) / len(self.latest_delays)

    # ------------------------------------------------------------------
    # Send JSON helper
    # ------------------------------------------------------------------

    def _send_json(self, msg: Dict[str, Any]) -> None:
        """Send a JSON command over the TCP socket."""
        if not self._sock:
            raise ConnectionError("bridge socket unavailable")
        data = json.dumps(msg, separators=(",", ":")) + "\n"
        try:
            self._sock.sendall(data.encode("utf-8"))
        except BlockingIOError:
            # Buffer full: drop the command (best effort)
            pass

    # ------------------------------------------------------------------
    # Close
    # ------------------------------------------------------------------

    def _close(self) -> None:
        self._connected = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def __del__(self) -> None:
        self.close()