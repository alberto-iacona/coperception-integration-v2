# Architecture Decisions — CARLA + CoPerception + OMNeT++ + Konro

**Project:** Master Thesis — Closed-loop network-aware cooperative perception with run-time resource management  
**Language:** English  
**Created:** 2026-11-11  
**Status:** Active record

## Purpose

This file documents every major architectural decision made during the project, including the rationale, alternatives considered, and consequences. It serves as the reference for the thesis methodology chapter.

---

## AD-001: Use CARLA BehaviorAgent subclass instead of Traffic Manager for controlled vehicles

**Decision:** Controlled (ego/partner) vehicles use a `CooperativeAgent` subclass of `carla.agents.navigation.BehaviorAgent`. Traffic Manager is retained only for background NPC/VRU traffic.

**Rationale:**
- Traffic Manager runs server-side in C++ using privileged map coordinates and oracle state data; it does not rely on sensor perception.
- Using Traffic Manager for controlled vehicles would violate the hard constraint: "vehicular decision-making must rely strictly on detected 3D bounding boxes from local/cooperative perception, not privileged CARLA ground-truth data."
- `BehaviorAgent` runs client-side in Python and exposes modular planners and PID controllers. Its hazard detection routine can be intercepted and replaced with CoPerception fused bounding boxes.

**Alternatives considered:**
- Rewrite CARLA vehicle dynamics and PID from scratch: rejected (unnecessary complexity, CARLA's PID works reliably).
- Use Traffic Manager with perception override: rejected (privileged data leakage).

**Consequences:**
- All controlled vehicles must run `CooperativeAgent` logic on the Python client.
- Background traffic remains under Traffic Manager (privileged data is acceptable for NPCs since they are not the subject of analysis).

---

## AD-002: Synchronous simulation mode with fixed_delta_seconds

**Decision:** CARLA world runs in synchronous mode (`synchronous_mode=True`) with a fixed time step (e.g., `fixed_delta_seconds = 0.05` for 20 Hz).

**Rationale:**
- Determinism is required for thesis reproducibility: identical runs with the same seed must produce identical results.
- Asynchronous mode introduces variable frame timing, which breaks the alignment between perception frames, network delays (OMNeT++), and resource allocation ticks (Konro).
- Fixed delta seconds enable precise Age of Information (AoI) and frame-lag calculations.

**Consequences:**
- The Python client must call `world.tick()` explicitly to advance the simulation.
- Real-time wall-clock performance is sacrificed for determinism (acceptable for offline thesis experiments).

---

## AD-003: CoPerception fusion model selection

**Decision:** Use intermediate fusion (feature-level) with models such as DiscoNet or V2VNet from the existing CoPerception SDK.

**Rationale:**
- The CoPerception SDK already implements DiscoNet, V2VNet, When2com, Who2com with pre-trained checkpoints.
- Intermediate fusion is the natural fit for live sensor streams: each vehicle extracts features from its LiDAR/camera, shares feature maps, and fuses them before bounding box prediction.
- Late fusion (box-level) would require additional calibration; early fusion (raw data) would require excessive bandwidth.

**Consequences:**
- Sensor adapter must output tensors in the format expected by CoPerception feature extractors.
- Checkpoint files (`.pth`) must be available for the selected model.

---

## AD-004: No ground-truth privileged data for driving decisions

**Decision:** Driving decisions (steering, throttle, brake) are based exclusively on 3D bounding boxes produced by CoPerception fusion.

**Rationale:**
- Hard constraint from the thesis architecture: realism requires that the vehicle "sees" only what its sensors and cooperative partners report.
- Privileged CARLA data (actor positions, velocities, bounding boxes from the simulator) must not leak into the control loop.

**Consequences:**
- If perception fails or network drops packets, the vehicle must physically react (emergency braking, increased TTC margin, or collision). This is a feature, not a bug.

---

## AD-005: Headless CARLA server

**Decision:** CARLA server launched with `-RenderOffScreen -nosound`.

**Rationale:**
- The workstation (RTX 3090 Ti) is dedicated to computation; rendering is unnecessary for perception and control logic.
- Headless mode avoids GPU contention between rendering and neural inference.
- Software rasterization (LLVMpipe) was discarded: 1-2 FPS invalidates synchronization with real-time neural networks.

**Consequences:**
- No visual output from CARLA; all debugging must be done via logs, saved sensor images, and JSON metrics.

---

## AD-006: Parametrized scenario design (N vehicles, variable maps)

**Decision:** TASK-01 code is written with configurable parameters: number of vehicles, map name, sensor suite, ego vehicle selection.

**Rationale:**
- The thesis requires testing multiple scenarios and vehicle counts, not just a single 2-vehicle Town03 test.
- Hardcoding 2 vehicles or Town03 would require rewriting code for each experiment.

**Consequences:**
- Initial validation uses 2 vehicles on Town03, but the same scripts scale to N vehicles and other maps.

## AD-007: Konro allocates PUs based on real data, not ground truth

**Decision:** Konro allocates Processing Units (PUs) based on proxy quality = fused_boxes / max_expected_boxes, where max_expected_boxes is a configurable estimate (not ground truth about total vehicles in scene). Network quality (packet loss, AoI) from OMNeT++ is also factored in.

**Rationale:**
- A real vehicle cannot know how many vehicles exist in the scene (ground truth).
- A real vehicle can observe: how many cooperative boxes it receives vs how many it could expect, packet loss ratio, Age of Information.
- Konro allocates more PUs when perception degrades AND network is good (more compute can help).
- Konro keeps baseline PUs when network is poor (more compute won't help if packets don't arrive).
- Konro decreases PUs when perception quality is high (energy savings).

**Consequences:**
- `konro_integration.py` computes proxy = num_fused_boxes / max_expected_boxes, not based on ground truth actor count.
- `konro_integration.py` reads `packet_loss_ratio` and `aois` from OMNeT++ bridge.
- Konro decision matrix: perception quality AND network quality both factor into PU allocation.

## AD-008: Konro-allocated PUs affect vehicle perception processing delay

**Decision:** CooperativeAgent uses Konro-allocated PUs to reduce processing delay: `effective_delay = max(0, sim_delay - (pus - baseline_pus) * 0.05)`. More PUs = less processing delay = fresher cooperative data = better perception quality.

**Rationale:**
- More CPU cores = faster feature extraction = less processing lag = fresher cooperative data.
- This creates a true closed-loop: perception quality -> Konro feedback -> PU allocation -> reduced processing delay -> fresher data -> better perception -> Konro reacts.

**Consequences:**
- `cooperative_agent.py` uses `self.konro_pus` to reduce effective network delay.
- `run_closed_loop.py` passes `konro_pus` to `CooperativeAgent` each frame.
- If Konro allocates 0 PUs (baseline), processing delay is maximum (stale data).
- If Konro allocates max PUs, processing delay is minimized (freshest data).

## AD-009: No privileged CARLA ground truth in Konro feedback

**Decision:** Konro receives only observable metrics: fused_boxes count / max_expected_boxes (proxy quality), packet_loss_ratio, Age of Information. No ground truth about number of vehicles in scene, true positions, etc.

**Rationale:**
- A real vehicle cannot know how many vehicles exist in the scene.
- A real vehicle can observe: how many cooperative boxes it receives, packet loss, Age of Information.
- Using ground truth would make the system unrealistic and not transferable to real vehicles.

**Consequences:**
- `run_closed_loop.py` computes proxy as `num_fused_boxes / max_expected_boxes` where `max_expected_boxes` is a configurable estimate (number of other vehicles + pedestrians), NOT ground truth actor count.
- `konro_integration.py` does NOT use ground truth actor count for proxy calculation.
- `omnet_bridge.py` provides packet_loss_ratio and aois (Age of Information) from actual packet transmission/reception timestamps.

---

*End of ARCHITECTURE_DECISIONS.md*