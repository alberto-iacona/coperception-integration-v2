# TASK-01.2 — Scalability to N Vehicles and Multiple Maps (Phase 1.2)

**Project:** Master Thesis — CARLA + CoPerception + OMNeT++ + Konro Integration  
**Task ID:** TASK-01.2  
**Phase:** Phase 1 (CARLA + CoPerception) — Sub-phase 1.2  
**Status:** Specification (not yet implemented)  
**Language:** English  
**Owner:** Alberto  
**Created:** 2026-11-11

## 1. Objective

Generalize the TASK-01 codebase (which was limited to 2 vehicles and Town03) to support an **arbitrary number of vehicles (N)** and **multiple CARLA maps**. This is required because the thesis evaluates the system across different scenarios and vehicle counts, not just a single 2-vehicle Town03 test.

The hard constraint remains: driving decisions must rely **strictly on CoPerception fused bounding boxes**, never on privileged CARLA ground-truth data.

## 2. Scope

### In scope
- Parametrized number of vehicles `N` via CLI argument (`--num-vehicles`)
- Configurable CARLA map via CLI argument (`--map`, e.g. Town03, Town04, Town05)
- Selectable ego vehicle for performance analysis via CLI argument (`--ego-id`)
- LiDAR + RGB camera sensors attached to every spawned vehicle
- Cooperative fusion among all N vehicles (each vehicle fuses boxes from all partners)
- Synchronous simulation with `fixed_delta_seconds` for determinism
- JSON metrics logging for the selected ego vehicle

### Out of scope (deferred to later phases)
- OMNeT++ network simulation (Phase 2)
- Konro resource management (Phase 3)
- Real-world traffic scenarios with complex occlusion (beyond basic map geometry)

## 3. Detailed specification

### 3.1 CLI arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--num-vehicles` | 2 | Number of vehicles to spawn (N >= 1) |
| `--map` | Town03 | CARLA map to load |
| `--target-speed` | 30.0 | Target speed in km/h for all vehicles |
| `--ttc-threshold` | 3.0 | Time-to-Collision threshold in seconds |
| `--cooperation` | disabled | Enable cooperative perception fusion |
| `--no-cooperation` | disabled | Explicitly disable cooperation (ablation baseline) |
| `--duration` | 60.0 | Simulation duration in seconds |
| `--fixed-delta-seconds` | 0.05 | Fixed time step for synchronous mode |
| `--ego-id` | None | CARLA vehicle id for performance analysis (default: first spawned) |
| `--log-dir` | logs/carla_run | Output directory for JSON metrics |

### 3.2 Vehicles and sensors (parametrized)

- N vehicles are spawned at sequential spawn points from the map's spawn point list
- Each vehicle mounts:
  - 1 LiDAR sensor (ray_cast, configurable range)
  - 1 RGB camera (front-facing, configurable resolution)
- All sensors stream data via CARLA callbacks into the sensor adapter
- Raw buffers are converted to PyTorch tensors for CoPerception feature extraction

### 3.3 Cooperative fusion among N vehicles

- Each vehicle publishes its fused bounding boxes to the in-memory cooperative registry
- Each vehicle reads boxes from all other N-1 vehicles (partners)
- Fusion: local boxes + partner boxes are stacked into a single array per vehicle
- In Phase 2 this in-memory registry will be replaced by OMNeT++ network messages

### 3.4 Ego vehicle selection

- The ego vehicle is the subject of performance analysis
- By default, the first spawned vehicle is the ego
- The user can override with `--ego-id <carla_vehicle_id>`
- Only the ego vehicle's metrics are logged to JSON (perception status, control commands)

## 4. Deliverables (files to be modified)

| File | Action | Description |
|------|--------|-------------|
| `tools/carla/scenario_setup.py` | Modify | Parametrize vehicle count and map loading (already mostly generic) |
| `tools/carla/run_closed_loop.py` | Modify | Add CLI arguments for N vehicles, map, ego-id; log only ego metrics |
| `tools/carla/cooperative_agent.py` | Modify (minimal) | Ensure fusion works with N partners (already supports list of partner IDs) |
| `docs/thesis/TASK-01-2_N_Vehicles_Maps.md` | Create | This specification file |
| `docs/thesis/PROJECT_LOG.md` | Update | Record start of Phase 1.2 |

## 5. Success criteria

| Criterion | Pass condition |
|-----------|----------------|
| N vehicles spawned | All N vehicles spawn successfully at distinct spawn points |
| Sensors active | Every vehicle has LiDAR + camera streaming data |
| N-1 cooperation | Each vehicle fuses boxes from all other N-1 vehicles |
| Ego selection | `--ego-id` correctly selects the analysis vehicle |
| Determinism | Identical results across runs with same seed and fixed_delta_seconds |
| No ground-truth | Driving decisions use only CoPerception bounding boxes |

## 6. Generalization path (post-TASK-01.2)

After TASK-01.2 is validated, the same code must support:
- Scenario matrix generation (multiple maps × multiple vehicle counts)
- Batch execution of experiments with automatic result collection
- Comparative analysis between cooperation ON and OFF conditions

---

*End of TASK-01-2_N_Vehicles_Maps.md*