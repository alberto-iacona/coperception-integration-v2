# TASK-01 — CARLA Sensor Harness & CoPerception Closed-Loop Agent (Phase 1)

**Project:** Master Thesis — CARLA + CoPerception + OMNeT++ + Konro Integration  
**Task ID:** TASK-01  
**Phase:** Phase 1 (CARLA + CoPerception)  
**Status:** Specification (not yet implemented)  
**Language:** English  
**Owner:** Alberto  
**Created:** 2026-11-11

## 1. Objective

Validate that the pipeline **CARLA → live sensors → CoPerception feature extraction → cooperative fusion → closed-loop vehicle control** works correctly, using **exclusively perception data** (fused 3D bounding boxes) for driving decisions. No privileged CARLA ground-truth data may be used for ego-vehicle control.

This task is the **minimal test bench** (2 vehicles, Town03 occluded intersection), but the code must be **generalizable**: parametrized number of vehicles, configurable maps, configurable sensors, and selectable ego vehicle for performance analysis.

## 2. Scope

### In scope
- CARLA server launched in headless mode (`-RenderOffScreen -nosound`)
- Synchronous simulation mode with `fixed_delta_seconds` for determinism
- Scenario setup: N vehicles spawned with LiDAR + RGB camera sensors
- Sensor data callback pipeline converting raw CARLA buffers into PyTorch tensors for CoPerception
- `CooperativeAgent` subclassing `carla.agents.navigation.BehaviorAgent`
- Hazard detection replaced by CoPerception fused 3D bounding boxes
- Closed-loop reaction: emergency braking, target speed reduction, or increased TTC margin when perception degrades
- Two test conditions: cooperation ON (ego avoids occluded obstacle) vs cooperation OFF (ego collides or brakes too late)

### Out of scope (deferred to later phases)
- OMNeT++ network simulation (Phase 2 / TASK-04)
- Konro resource management and energy modeling (Phase 3 / TASK-05)
- Large-scale scenario matrix with N > 2 vehicles (Phase 1.2+)

## 3. Detailed specification

### 3.1 Environment setup

| Item | Specification |
|------|---------------|
| CARLA version | 0.9.15 |
| Server path | `/home/alberto/carla/` |
| Launcher | `./CarlaUE4.sh -RenderOffScreen -nosound -world-port=2000` |
| Python venv | `/home/alberto/coperception-Integration/venv/` |
| Map | `Town03` (initial test) |
| Simulation mode | Synchronous (`world.apply_settings(synchronous_mode=True)`) |
| Fixed delta seconds | Configurable (e.g., 0.05s for 20 Hz) |
| Port | 2000 (configurable) |

### 3.2 Vehicles and sensors (parametrized)

- Number of vehicles: `N` (default 2 for initial test: 1 ego + 1 partner)
- Each vehicle mounts:
  - 1 LiDAR sensor (point cloud)
  - 1 RGB camera (front-facing)
- Sensors are attached to each spawned actor and stream data via CARLA callbacks
- Raw buffers are converted to PyTorch tensors formatted for the CoPerception feature extractor

### 3.3 CooperativeAgent (subclass of BehaviorAgent)

The agent must:

1. At each `world.tick()`:
   - Read the most recent tensors from its own LiDAR and camera (local perception)
   - Run CoPerception feature extraction on local data
   - Fuse local features with partner features (intermediate fusion, e.g. DiscoNet / V2VNet)
   - Produce fused 3D bounding boxes in the ego vehicle reference frame
2. Replace `BehaviorAgent` hazard detection:
   - Evaluate collision risk **only** using fused bounding boxes
   - Do NOT query CARLA ground-truth positions of other actors for driving decisions
3. Closed-loop control:
   - If TTC < threshold or obstacle on trajectory: apply emergency brake, reduce target speed, or increase TTC margin
   - If perception is missing or degraded: react physically (late braking or collision) to demonstrate the closed-loop safety property

### 3.4 Scenario: Town03 occluded T-junction

- Ego vehicle approaches an intersection where cross traffic is occluded by a building
- Partner vehicle has clear line of sight to the occluded area
- An obstacle / VRU is hidden from the ego's direct view but visible to the partner
- Condition A (cooperation ON): ego receives partner's fused boxes and avoids the obstacle
- Condition B (cooperation OFF): ego relies only on local sensors, collides or brakes too late

## 4. Deliverables (files to be created)

| File | Description |
|------|-------------|
| `tools/carla/scenario_setup.py` | CARLA connection, map loading, vehicle/sensor spawning, synchronous mode |
| `tools/carla/sensor_adapter.py` | Raw CARLA buffer → PyTorch tensor conversion for CoPerception |
| `tools/carla/cooperative_agent.py` | `CooperativeAgent(BehaviorAgent)` with perception-based hazard detection |
| `tools/carla/run_closed_loop.py` | Main orchestration script: scenario + sensors + agents + simulation loop |

## 5. Success criteria

| Criterion | Pass condition |
|-----------|----------------|
| Pipeline connectivity | CARLA server accepts Python client connection; sensors stream data |
| Tensor conversion | LiDAR / camera buffers convert to valid PyTorch tensors without shape errors |
| CoPerception inference | Feature extraction and fusion run on live tensors (not dataset) |
| Closed-loop driving | Ego vehicle avoids occluded obstacle when cooperation is ON |
| No ground-truth usage | Driving decisions use only CoPerception bounding boxes |
| Determinism | Identical results across runs with same seed and `fixed_delta_seconds` |
| Failure mode | Without cooperation, ego collides or brakes too late (validates dependence on cooperative data) |

## 6. Generalization path (post-TASK-01)

After TASK-01 is validated, the same code must support:
- N vehicles (configurable, not hardcoded to 2)
- Multiple maps (Town01, Town03, Town04, Town05, etc.)
- Selectable ego vehicle for performance analysis
- Scenario matrix generation for experimental campaigns

---

*End of TASK-01_CARLA_CoPerception.md*