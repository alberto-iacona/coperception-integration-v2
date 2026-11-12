# Future Features Backlog — CARLA + CoPerception + OMNeT++ + Konro

**Project:** Master Thesis — Closed-loop network-aware cooperative perception with run-time resource management  
**Document Type:** Feature backlog / enhancement roadmap  
**Language:** English  
**Created:** 2026-11-12  
**Status:** Active record

## 1. Purpose

This document collects **all possible features and enhancements** that could be added to the system at any phase of development. Items are not tied to a specific phase; they are recorded here so they are not forgotten during development. During each phase, the developer can review this list and decide whether to:
- Implement now (if critical for the current phase)
- Defer to a later phase
- Discard (if out of scope for the thesis)

Each item includes: description, rationale, suggested phase, effort estimate, and dependencies.

---

## 2. Phase 1 — CARLA + CoPerception (baseline closed-loop)

### PF-01: Real CoPerception inference instead of synthetic boxes
**Description:** Replace the `_simulate_coperception_output()` placeholder in `run_closed_loop.py` with actual CoPerception model inference (DiscoNet / V2VNet / When2com) on live LiDAR/camera tensors from CARLA.  
**Rationale:** TASK-01 currently uses synthetic boxes ahead of the vehicle. Real inference is needed to validate that CoPerception works on live CARLA sensor data.  
**Suggested phase:** Phase 1.2 or 1.3  
**Effort:** High (requires loading checkpoints, preprocessing CARLA tensors to CoPerception format)  
**Dependencies:** Checkpoint `.pth` files, CoPerception model code in `coperception/models/det/`

### PF-02: Multiple sensor configurations per vehicle
**Description:** Allow configuring different sensor suites per vehicle (e.g., LiDAR only, camera only, LiDAR + camera, multi-camera).  
**Rationale:** Different vehicles may have different compute capabilities; sensor configuration affects perception quality and bandwidth.  
**Suggested phase:** Phase 1.2+  
**Effort:** Medium  
**Dependencies:** `scenario_setup.py`, `sensor_adapter.py`

### PF-03: Dynamic spawn point generation
**Description:** Instead of sequential spawn points, generate random or scenario-specific spawn points (e.g., vehicles approaching an intersection from different directions).  
**Rationale:** More realistic traffic scenarios; avoids vehicles spawning too close to each other.  
**Suggested phase:** Phase 1.2+  
**Effort:** Low-Medium  
**Dependencies:** `scenario_setup.py`

### PF-04: Scenario configuration file (YAML/JSON)
**Description:** Move scenario parameters (map, num_vehicles, spawn points, sensor suites, target speeds) to a configuration file instead of CLI arguments.  
**Rationale:** Easier to run batch experiments with many scenario combinations.  
**Suggested phase:** Phase 1.2+  
**Effort:** Low  
**Dependencies:** `run_closed_loop.py`

### PF-05: Ground-truth comparison logging (for analysis only)
**Description:** Log CARLA privileged ground-truth positions alongside perception boxes for offline analysis (not used for driving decisions).  
**Rationale:** Enables quantitative comparison of perception error vs. ground truth for the thesis.  
**Suggested phase:** Phase 1.2+  
**Effort:** Low  
**Dependencies:** `run_closed_loop.py` metrics logging

---

## 3. Phase 2 — OMNeT++ Network Injection

### PF-06: Building / obstacle shadowing in OMNeT++
**Description:** Add obstacle-aware propagation (buildings, palaces) so vehicles separated by obstacles experience additional attenuation. See `OMNET_LIMITATIONS_AND_ENHANCEMENTS.md`.  
**Rationale:** Makes network loss depend on the physical scene, not just distance. Critical for urban occlusion scenarios.  
**Suggested phase:** Phase 2 or 3  
**Effort:** High  
**Dependencies:** `omnetpp.ini`, `CoSim.ned`, INET propagation models, possibly CARLA building geometry export

### PF-07: Fading / multipath models
**Description:** Add Rayleigh/Rician fading or LogNormal shadowing to the radio medium.  
**Rationale:** More realistic small-scale fading in urban environments.  
**Suggested phase:** Phase 2 or 3  
**Effort:** Medium  
**Dependencies:** `omnetpp.ini`

### PF-08: Explicit channel saturation model
**Description:** Add configurable packet drop probability based on local vehicle density or channel load.  
**Rationale:** Study what happens when many vehicles transmit simultaneously in a dense area.  
**Suggested phase:** Phase 2 or 3  
**Effort:** Medium  
**Dependencies:** `omnetpp.ini`, `NetworkManager.cc`

### PF-09: Multiple OMNeT++ connections (one bridge per vehicle)
**Description:** Instead of a single central bridge, give each vehicle its own TCP connection to OMNeT++.  
**Rationale:** More realistic (each vehicle has its own radio interface); avoids single point of failure. Requires OMNeT++ to accept multiple connections.  
**Suggested phase:** Phase 2 or later  
**Effort:** High  
**Dependencies:** `NetworkManager.cc` (accept multiple clients), `omnet_bridge.py`

### PF-10: V2I (Vehicle-to-Infrastructure) communication
**Description:** Add RSU (Road Side Unit) nodes in OMNeT++ that can relay messages between vehicles.  
**Rationale:** V2I can extend range and reduce packet loss in occluded scenarios.  
**Suggested phase:** Phase 2 or 3  
**Effort:** Medium-High  
**Dependencies:** `CoSim.ned` (add RSU nodes), `omnet_bridge.py`

---

## 4. Phase 3 — Konro Resource Management & Energy

### PF-11: Automotive SoC power model
**Description:** Implement `P_compute(t) = P_idle + sum(P_core * U_core) + P_NPU` mapping Konro PUs to CPU cores and NPU inference to power draw.  
**Rationale:** Core thesis contribution: quantify energy/compute trade-offs under degraded V2X.  
**Suggested phase:** Phase 3  
**Effort:** High  
**Dependencies:** Konro cgroups, CPU utilization monitoring, power model parameters (Thor/HW4-like SoC)

### PF-12: Konro feedback from perception quality (EMA of recall/proxy)
**Description:** Use the proxy metric (recall × FP penalty) already computed in `konro_bridge.py` as feedback to Konro for PU allocation.  
**Rationale:** When perception degrades (low recall), Konro allocates more PUs to speed up inference; when network is the bottleneck, Konro drops to 1 PU to save energy.  
**Suggested phase:** Phase 3  
**Effort:** Medium  
**Dependencies:** `konro_bridge.py`, `PerceptionProxyTracker`, Konro running on localhost:8080

### PF-13: Age of Information (AoI) tracking
**Description:** Track AoI for each cooperative data stream (time since last successfully received box from each partner).  
**Rationale:** AoI is a key KPI for network-aware perception; high AoI means stale data is being used for driving decisions.  
**Suggested phase:** Phase 3  
**Effort:** Low-Medium  
**Dependencies:** `omnet_bridge.py` (received timestamps), metrics logging

### PF-14: Timeout / radio blackout handling
**Description:** If OMNeT++ reports no successful deliveries for N consecutive frames, trigger Konro timeout (drop to 1 PU baseline) and fallback to local-only perception.  
**Rationale:** Safety-critical: when V2X is disconnected, the vehicle must conserve energy and rely on local sensors.  
**Suggested phase:** Phase 3  
**Effort:** Medium  
**Dependencies:** Konro, `omnet_bridge.py`, `cooperative_agent.py`

### PF-15: Energy consumption logging (Joules per frame)
**Description:** Log instantaneous power and cumulative energy consumption per vehicle per frame.  
**Rationale:** Enables Pareto analysis: perception quality vs. energy consumption.  
**Suggested phase:** Phase 3  
**Effort:** Medium  
**Dependencies:** Power model (PF-11), metrics logging

---

## 5. Phase 4 — Evaluation Campaign & KPI Analysis

### PF-16: Batch experiment runner
**Description:** Script that automatically runs multiple scenarios (different maps, vehicle counts, network conditions, cooperation on/off) and collects all metrics.  
**Rationale:** Thesis requires statistical comparison across many runs, not just manual tests.  
**Suggested phase:** Phase 4  
**Effort:** Medium  
**Dependencies:** `run_closed_loop.py` CLI, scenario config files (PF-04)

### PF-17: Comparative analysis: Local vs. Cooperative vs. Cooperative+Konro
**Description:** Benchmark three conditions: (1) local sensors only, (2) cooperative V2X without resource management, (3) cooperative V2X with Konro.  
**Rationale:** Core thesis evaluation: does Konro add value beyond raw cooperation?  
**Suggested phase:** Phase 4  
**Effort:** Medium  
**Dependencies:** All previous phases working

### PF-18: Safety/comfort KPIs (TTC, stopping distance, jerk, emergency brake count)
**Description:** Compute and log vehicular safety and comfort metrics from CARLA vehicle dynamics.  
**Rationale:** The thesis must show that cooperative perception improves safety (or degrades gracefully when network fails).  
**Suggested phase:** Phase 4  
**Effort:** Low-Medium  
**Dependencies:** CARLA vehicle telemetry, metrics logging

### PF-19: Multi-map evaluation matrix
**Description:** Run the same experiment on Town03, Town04, Town05, etc. to show generalization across urban layouts.  
**Rationale:** Demonstrates the system is not tuned to a single scenario.  
**Suggested phase:** Phase 4  
**Effort:** Low  
**Dependencies:** `--map` CLI arg (already implemented)

### PF-20: N-vehicle scalability test (N > 6)
**Description:** Test with 10, 20, 50 vehicles to evaluate network saturation and Konro behavior under load.  
**Rationale:** Shows whether the system scales beyond the minimal 2-vehicle test.  
**Suggested phase:** Phase 4  
**Effort:** Medium  
**Dependencies:** OMNeT++ performance, Konro scalability

---

## 5. Cross-Phase / Infrastructure Features

### PF-21: Configuration management (YAML/JSON for all parameters)
**Description:** Central configuration file for all experiment parameters (CARLA, OMNeT++, Konro, CoPerception, logging).  
**Rationale:** Reproducibility is essential for a thesis.  
**Suggested phase:** Any (early recommended)  
**Effort:** Low  
**Dependencies:** None

### PF-22: Automated result visualization (Python plotting scripts)
**Description:** Scripts to generate comparison charts (recall vs. stopping distance, energy vs. TTC, etc.) from JSON metrics.  
**Rationale:** Thesis figures need to be generated from experimental data.  
**Suggested phase:** Phase 4  
**Effort:** Low-Medium  
**Dependencies:** JSON metrics files, matplotlib

### PF-23: Deterministic seed management
**Description:** Ensure all random sources (CARLA, OMNeT++, Python, PyTorch) use fixed seeds for reproducible runs.  
**Rationale:** Thesis experiments must be reproducible.  
**Suggested phase:** Any  
**Effort:** Low  
**Dependencies:** `omnetpp.ini` (seed-set = 42 already), Python `random`, `torch.manual_seed`

### PF-24: Docker / containerization for reproducibility
**Description:** Create Dockerfile with CARLA, OMNeT++, Python dependencies pre-installed.  
**Rationale:** Easy replication on other machines.  
**Suggested phase:** Optional / end of thesis  
**Effort:** High  
**Dependencies:** Docker, CARLA license, OMNeT++ license

---

## 6. How to Use This Document

1. **At the start of each phase:** Review this backlog and select items relevant to that phase.
2. **During development:** If you think of a feature not listed here, add it with description, rationale, and suggested phase.
3. **Before marking a phase complete:** Check that no high-priority backlog item was skipped without justification.
4. **Item status:** Items are not assigned to specific commits; they are a menu of options. The thesis author decides what to implement based on time and relevance.

---

*End of FUTURE_FEATURES_BACKLOG.md*