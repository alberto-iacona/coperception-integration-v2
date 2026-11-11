"""
Scenario setup module for CARLA + CoPerception integration (TASK-01, Phase 1).

This module handles the CARLA server connection, map loading, synchronous
simulation configuration, and spawning of vehicles with sensor suites.
It is parametrized so that the number of vehicles, map name, and sensor
configuration can be changed without modifying the code.

Responsibilities:
    1. Connect to the CARLA server (headless mode).
    2. Load a configurable map (default: Town03).
    3. Enable synchronous mode with fixed_delta_seconds for determinism.
    4. Spawn N vehicles at configurable spawn points.
    5. Attach LiDAR and RGB camera sensors to each vehicle.
    6. Return the world, client, vehicles, and sensors for the control loop.

Hard constraints respected:
    - No privileged CARLA ground-truth is used for driving decisions.
    - All vehicles run in synchronous mode for reproducible experiments.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional, Tuple

import carla  # type: ignore

logger = logging.getLogger(__name__)


class ScenarioSetup:
    """
    Encapsulates CARLA world setup and actor spawning for cooperative
    perception experiments.

    Attributes:
        client: CARLA client connection.
        world: CARLA world object.
        map_name: Name of the loaded CARLA map.
        vehicles: List of spawned vehicle actors.
        sensors: Dictionary mapping vehicle id to its sensor actors.
        settings_original: Original world settings (for restoration on cleanup).
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 2000,
        timeout: float = 10.0,
        map_name: str = "Town03",
        fixed_delta_seconds: float = 0.05,
        synchronous_mode: bool = True,
    ) -> None:
        """
        Initialize the CARLA client and connect to the server.

        Args:
            host: IP address of the CARLA server.
            port: TCP port of the CARLA server.
            timeout: Connection timeout in seconds.
            map_name: CARLA map to load (e.g. "Town03", "Town04").
            fixed_delta_seconds: Fixed time step for synchronous simulation.
            synchronous_mode: Whether to enable synchronous mode.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.map_name = map_name
        self.fixed_delta_seconds = fixed_delta_seconds
        self.synchronous_mode = synchronous_mode

        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.vehicles: List[carla.Vehicle] = []
        self.sensors: Dict[int, List[carla.Sensor]] = {}
        self.settings_original: Optional[Any] = None

        self._connect()
        self._load_map()
        self._configure_world()

    def _connect(self) -> None:
        """Establish the TCP connection to the CARLA server."""
        logger.info("Connecting to CARLA server at %s:%d...", self.host, self.port)
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        logger.info("CARLA client connected.")

    def _load_map(self) -> None:
        """Load the specified CARLA map."""
        assert self.client is not None
        logger.info("Loading map: %s", self.map_name)
        self.world = self.client.load_world(self.map_name)
        logger.info("Map %s loaded.", self.map_name)

    def _configure_world(self) -> None:
        """
        Configure the world for deterministic synchronous simulation.

        Stores the original settings so they can be restored during cleanup.
        Enables synchronous mode and sets a fixed delta seconds for
        reproducible frame timing across runs.
        """
        assert self.world is not None
        self.settings_original = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = self.synchronous_mode
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        self.world.apply_settings(settings)
        logger.info(
            "World configured: synchronous=%s, fixed_delta_seconds=%.3f",
            self.synchronous_mode,
            self.fixed_delta_seconds,
        )

    def spawn_vehicle(
        self,
        spawn_index: int = 0,
        vehicle_filter: str = "vehicle.*",
        role_name: str = "ego",
    ) -> carla.Vehicle:
        """
        Spawn a single vehicle at a given spawn point index.

        Args:
            spawn_index: Index into the map's spawn points list.
            vehicle_filter: Blueprint filter for the vehicle type.
            role_name: Role name attribute for the vehicle (useful for logging).

        Returns:
            The spawned CARLA vehicle actor.

        Raises:
            RuntimeError: If no spawn points are available or spawning fails.
        """
        assert self.world is not None
        blueprint_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()

        if not spawn_points:
            raise RuntimeError("No spawn points available on map %s" % self.map_name)

        if not spawn_points:
            raise RuntimeError("No spawn points available on map %s" % self.map_name)

        # Use cyclic indexing when N exceeds the number of available spawn points.
        # This allows spawning more vehicles than there are predefined spawn points
        # by reusing locations with a small offset to avoid immediate collisions.
        effective_index = spawn_index % len(spawn_points)
        if spawn_index >= len(spawn_points):
            logger.warning(
                "Spawn index %d out of range (%d points). Using cyclic index %d.",
                spawn_index,
                len(spawn_points),
                effective_index,
            )

        bp = blueprint_library.filter(vehicle_filter)[0]
        bp.set_attribute("role_name", role_name)

        transform = spawn_points[effective_index]
        # Apply a small longitudinal offset for cyclically reused spawn points
        # to reduce the chance of immediate overlap between vehicles.
        if spawn_index >= len(spawn_points):
            transform.location.z += 0.5  # slight height offset to avoid collision

        vehicle = self.world.spawn_actor(bp, transform)
        self.vehicles.append(vehicle)

        logger.info(
            "Spawned vehicle %s at spawn point %d (effective=%d, id=%d, transform=%s)",
            role_name,
            spawn_index,
            effective_index,
            vehicle.id,
            transform,
        )
        return vehicle

    def spawn_vehicles(
        self,
        num_vehicles: int = 2,
        vehicle_filter: str = "vehicle.*",
        role_names: Optional[List[str]] = None,
    ) -> List[carla.Vehicle]:
        """
        Spawn multiple vehicles at sequential spawn points.

        Args:
            num_vehicles: Number of vehicles to spawn.
            vehicle_filter: Blueprint filter for the vehicle type.
            role_names: Optional list of role names (one per vehicle).
                        If shorter than num_vehicles, defaults are used.

        Returns:
            List of spawned CARLA vehicle actors.
        """
        if role_names is None:
            role_names = [f"vehicle_{i}" for i in range(num_vehicles)]
        elif len(role_names) < num_vehicles:
            role_names = role_names + [f"vehicle_{i}" for i in range(len(role_names), num_vehicles)]

        vehicles: List[carla.Vehicle] = []
        for i in range(num_vehicles):
            vehicle = self.spawn_vehicle(
                spawn_index=i,
                vehicle_filter=vehicle_filter,
                role_name=role_names[i],
            )
            vehicles.append(vehicle)

        logger.info("Spawned %d vehicles.", len(vehicles))
        return vehicles

    def attach_sensors(
        self,
        vehicle: carla.Vehicle,
        enable_lidar: bool = True,
        enable_camera: bool = True,
        lidar_range: float = 50.0,
        lidar_points_per_second: int = 100000,
        camera_width: int = 800,
        camera_height: int = 600,
        camera_fov: float = 90.0,
        sensor_transform_z: float = 2.0,
    ) -> List[carla.Sensor]:
        """
        Attach LiDAR and/or RGB camera sensors to a vehicle.

        Sensors are mounted at a fixed offset above the vehicle (z = sensor_transform_z)
        with no rotation (facing forward). Each sensor registers a callback that
        stores the latest measurement for retrieval by the control loop.

        Args:
            vehicle: The vehicle actor to attach sensors to.
            enable_lidar: Whether to attach a ray-cast LiDAR sensor.
            enable_camera: Whether to attach an RGB camera sensor.
            lidar_range: Maximum detection range of the LiDAR in meters.
            lidar_points_per_second: Number of points per second for the LiDAR.
            camera_width: Image width in pixels.
            camera_height: Image height in pixels.
            camera_fov: Field of view in degrees for the camera.
            sensor_transform_z: Height in meters above the vehicle base.

        Returns:
            List of attached sensor actors.
        """
        assert self.world is not None
        blueprint_library = self.world.get_blueprint_library()
        attached: List[carla.Sensor] = []

        # Mounting transform: forward-facing, above the vehicle roof
        transform = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=sensor_transform_z)
        )

        if enable_lidar:
            lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
            lidar_bp.set_attribute("range", str(lidar_range))
            lidar_bp.set_attribute("points_per_second", str(lidar_points_per_second))
            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
            lidar.listen(self._make_lidar_callback(vehicle.id))
            attached.append(lidar)
            logger.info("Attached LiDAR to vehicle %d.", vehicle.id)

        if enable_camera:
            camera_bp = blueprint_library.find("sensor.camera.rgb")
            camera_bp.set_attribute("image_size_x", str(camera_width))
            camera_bp.set_attribute("image_size_y", str(camera_height))
            camera_bp.set_attribute("fov", str(camera_fov))
            camera = self.world.spawn_actor(camera_bp, transform, attach_to=vehicle)
            camera.listen(self._make_camera_callback(vehicle.id))
            attached.append(camera)
            logger.info("Attached RGB camera to vehicle %d.", vehicle.id)

        self.sensors.setdefault(vehicle.id, []).extend(attached)
        return attached

    def _make_lidar_callback(self, vehicle_id: int):
        """Create a callback that stores the latest LiDAR measurement for a vehicle."""
        latest: Dict[str, Any] = {"data": None, "timestamp": None}

        def callback(measurement: carla.LidarMeasurement) -> None:
            latest["data"] = measurement
            latest["timestamp"] = measurement.timestamp

        # Store the reference on the callback object for external access
        callback.latest = latest  # type: ignore[attr-defined]
        return callback

    def _make_camera_callback(self, vehicle_id: int):
        """Create a callback that stores the latest camera image for a vehicle."""
        latest: Dict[str, Any] = {"data": None, "timestamp": None}

        def callback(image: carla.Image) -> None:
            latest["data"] = image
            latest["timestamp"] = image.timestamp

        callback.latest = latest  # type: ignore[attr-defined]
        return callback

    def get_latest_sensor_data(self, vehicle_id: int) -> Dict[str, Any]:
        """
        Retrieve the latest sensor measurements for a given vehicle.

        Args:
            vehicle_id: The CARLA id of the vehicle.

        Returns:
            Dictionary with keys "lidar" and/or "camera", each containing
            {"data": measurement, "timestamp": float} or None if unavailable.
        """
        result: Dict[str, Any] = {}
        sensors = self.sensors.get(vehicle_id, [])
        for sensor in sensors:
            callback = getattr(sensor, "callback_list", None)
            if callback is None:
                # Fallback: inspect the listener if stored as attribute
                continue
            sensor_type = sensor.type_id
            if "lidar" in sensor_type:
                result["lidar"] = getattr(callback, "latest", None)
            elif "camera" in sensor_type:
                result["camera"] = getattr(callback, "latest", None)
        return result

    def tick(self) -> None:
        """Advance the synchronous simulation by one time step."""
        assert self.world is not None
        self.world.tick()

    def cleanup(self) -> None:
        """
        Destroy all spawned actors and restore original world settings.

        Should be called at the end of the simulation to avoid leaving
        orphaned actors in the CARLA world.
        """
        logger.info("Cleaning up scenario: destroying %d vehicles and sensors...", len(self.vehicles))

        # Destroy sensors first
        for vehicle_id, sensor_list in self.sensors.items():
            for sensor in sensor_list:
                if sensor is not None and sensor.is_alive:
                    sensor.destroy()
            logger.debug("Destroyed sensors for vehicle %d.", vehicle_id)

        # Destroy vehicles
        for vehicle in self.vehicles:
            if vehicle is not None and vehicle.is_alive:
                vehicle.destroy()
        self.vehicles.clear()
        self.sensors.clear()

        # Restore original world settings
        if self.world is not None and self.settings_original is not None:
            self.world.apply_settings(self.settings_original)
            logger.info("Original world settings restored.")

        logger.info("Cleanup complete.")
