"""
Sensor adapter module for CARLA + CoPerception integration (TASK-01, Phase 1).

This module converts raw CARLA sensor data (LiDAR point clouds and RGB camera
images) into PyTorch tensors formatted for consumption by the CoPerception
feature extractor.

Responsibilities:
    1. Convert CARLA LidarMeasurement raw bytes into a [N, 4] float32 tensor
       containing (x, y, z, intensity) per point.
    2. Convert CARLA Image raw bytes into a [3, H, W] float32 tensor containing
       normalized RGB pixels.
    3. Provide a unified interface to extract the latest sensor tensors from
       any vehicle equipped with sensors via ScenarioSetup.

Design notes:
    - LiDAR points are expressed in the vehicle's local coordinate frame.
    - Camera images are converted from CARLA's BGRA layout to PyTorch's RGB
      channel-first layout (C, H, W) with values normalized to [0, 1].
    - No ground-truth data is used: only raw sensor measurements are converted.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

import numpy as np
import torch  # type: ignore

logger = logging.getLogger(__name__)


class SensorAdapter:
    """
    Adapts CARLA raw sensor outputs to PyTorch tensors for CoPerception.

    Attributes:
        device: Torch device on which tensors are allocated.
    """

    def __init__(self, device: Optional[torch.device] = None) -> None:
        """
        Initialize the sensor adapter.

        Args:
            device: Torch device (e.g. torch.device('cuda') or torch.device('cpu')).
                    Defaults to CUDA if available, otherwise CPU.
        """
        if device is None:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = device
        logger.info("SensorAdapter initialized on device: %s", self.device)

    # ------------------------------------------------------------------
    # LiDAR conversion
    # ------------------------------------------------------------------

    @staticmethod
    def lidar_to_tensor(measurement: Any) -> torch.Tensor:
        """
        Convert a CARLA LidarMeasurement into a PyTorch tensor.

        The raw data buffer contains 32-bit floats. Depending on the CARLA
        version and LiDAR configuration, each point may be represented by
        3 floats (x, y, z) or 4 floats (x, y, z, intensity). This method
        detects the stride automatically.

        Args:
            measurement: A carla.LidarMeasurement object.

        Returns:
            A float32 tensor of shape [N, 4] where each row is
            (x, y, z, intensity). If intensity is unavailable, it is set to 1.0.

        Raises:
            ValueError: If the raw data size is not a multiple of 12 or 16 bytes.
        """
        raw_data = measurement.raw_data
        num_bytes = len(raw_data)

        # Each point is either 3 floats (12 bytes) or 4 floats (16 bytes)
        if num_bytes % 16 == 0:
            floats_per_point = 4
        elif num_bytes % 12 == 0:
            floats_per_point = 3
        else:
            raise ValueError(
                "LiDAR raw data size %d is not a multiple of 12 or 16 bytes" % num_bytes
            )

        num_points = num_bytes // (floats_per_point * 4)  # 4 bytes per float32
        points = np.frombuffer(raw_data, dtype=np.float32).reshape(num_points, floats_per_point)

        if floats_per_point == 3:
            # Append a column of ones for intensity
            intensity = np.ones((num_points, 1), dtype=np.float32)
            points = np.hstack([points, intensity])

        tensor = torch.from_numpy(points.copy()).to(dtype=torch.float32)
        logger.debug(
            "Converted LiDAR: %d points -> tensor shape %s", num_points, tuple(tensor.shape)
        )
        return tensor

    # ------------------------------------------------------------------
    # Camera conversion
    # ------------------------------------------------------------------

    @staticmethod
    def camera_to_tensor(image: Any) -> torch.Tensor:
        """
        Convert a CARLA Image into a PyTorch tensor.

        CARLA images are stored in BGRA format with shape [H, W, 4].
        This method converts to RGB channel-first layout [3, H, W] with
        pixel values normalized to [0, 1].

        Args:
            image: A carla.Image object.

        Returns:
            A float32 tensor of shape [3, H, W] with normalized RGB values.
        """
        height = image.height
        width = image.width
        raw_data = image.raw_data

        # CARLA stores BGRA: [H, W, 4] uint8
        array = np.frombuffer(raw_data, dtype=np.uint8).reshape(height, width, 4)

        # Drop alpha channel and convert BGR -> RGB
        bgr = array[:, :, :3]
        rgb = bgr[:, :, ::-1]  # reverse channels: BGR -> RGB

        # Normalize to [0, 1] and convert to channel-first [3, H, W]
        tensor = torch.from_numpy(rgb.copy()).to(dtype=torch.float32) / 255.0
        tensor = tensor.permute(2, 0, 1)  # [H, W, 3] -> [3, H, W]

        logger.debug(
            "Converted camera image: %dx%d -> tensor shape %s", width, height, tuple(tensor.shape)
        )
        return tensor

    # ------------------------------------------------------------------
    # Unified extraction
    # ------------------------------------------------------------------

    def extract_sensor_tensors(
        self,
        sensor_data: Dict[str, Any],
    ) -> Dict[str, Optional[torch.Tensor]]:
        """
        Extract PyTorch tensors from a dictionary of CARLA sensor measurements.

        The input dictionary is expected to be in the format returned by
        ScenarioSetup.get_latest_sensor_data(), i.e. keys "lidar" and/or
        "camera", each mapping to {"data": measurement, "timestamp": float}.

        Args:
            sensor_data: Dictionary of latest sensor measurements per vehicle.

        Returns:
            Dictionary with keys "lidar" and/or "camera", each containing a
            torch.Tensor on self.device, or None if that sensor is absent.
        """
        result: Dict[str, Optional[torch.Tensor]] = {
            "lidar": None,
            "camera": None,
        }

        lidar_entry = sensor_data.get("lidar")
        if lidar_entry is not None and lidar_entry.get("data") is not None:
            lidar_tensor = self.lidar_to_tensor(lidar_entry["data"]).to(self.device)
            result["lidar"] = lidar_tensor
            logger.debug("LiDAR tensor extracted: shape %s", tuple(lidar_tensor.shape))

        camera_entry = sensor_data.get("camera")
        if camera_entry is not None and camera_entry.get("data") is not None:
            camera_tensor = self.camera_to_tensor(camera_entry["data"]).to(self.device)
            result["camera"] = camera_tensor
            logger.debug("Camera tensor extracted: shape %s", tuple(camera_tensor.shape))

        return result

    def extract_all_vehicles(
        self,
        scenario_setup: Any,
        vehicle_ids: Optional[List[int]] = None,
    ) -> Dict[int, Dict[str, Optional[torch.Tensor]]]:
        """
        Extract sensor tensors for all (or a subset of) vehicles in the scenario.

        Args:
            scenario_setup: An instance of ScenarioSetup (or any object exposing
                            get_latest_sensor_data(vehicle_id)).
            vehicle_ids: List of vehicle CARLA ids. If None, uses all vehicles
                         known to scenario_setup.

        Returns:
            Dictionary mapping vehicle CARLA id -> sensor tensor dictionary.
        """
        if vehicle_ids is None:
            vehicle_ids = [v.id for v in getattr(scenario_setup, "vehicles", [])]

        all_tensors: Dict[int, Dict[str, Optional[torch.Tensor]]] = {}
        for vid in vehicle_ids:
            sensor_data = scenario_setup.get_latest_sensor_data(vid)
            all_tensors[vid] = self.extract_sensor_tensors(sensor_data)

        return all_tensors
