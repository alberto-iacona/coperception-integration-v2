"""
CARLA integration package for CoPerception closed-loop simulation.

This package contains the modules required to integrate CARLA as the
physical simulation plane with CoPerception for cooperative perception
and closed-loop vehicle control.

Modules:
    scenario_setup   : CARLA connection, map loading, vehicle/sensor spawning.
    sensor_adapter   : Conversion of raw CARLA sensor buffers to PyTorch tensors.
    cooperative_agent: BehaviorAgent subclass using CoPerception fused boxes.
    run_closed_loop  : Main orchestration script for the simulation loop.
"""
