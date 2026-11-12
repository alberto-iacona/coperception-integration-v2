# TASK-04 — Experimental Campaign & KPI Analysis (Phase 4)

**Project:** Master Thesis — CARLA + CoPerception + OMNeT++ + Konro Integration  
**Task ID:** TASK-04  
**Phase:** Phase 4 — Evaluation Campaign & KPI Analysis  
**Status:** Specification (not yet implemented)  
**Language:** English  
**Owner:** Alberto  
**Created:** 2026-11-12

## 1. Objective

Demonstrate, with quantitative and repeatable data, that the closed-loop integration
**CARLA + CoPerception + OMNeT++ + Konro** provides concrete benefits over baseline
configurations. The central thesis question is:

> **Does Konro add value beyond simple cooperative V2X?**

Phase 4 executes a battery of experiments across multiple conditions (cooperation ON/OFF,
Konro ON/OFF, multiple maps, multiple vehicle counts, degraded network) and analyzes
the resulting KPIs to evaluate system, network, energy, safety, and comfort performance.

## 2. Experimental Matrix

### 2.1 Conditions (three main scenarios)

| Condition | Description |
|-----------|-------------|
| **Scenario 1 — Baseline (Local Only)** | Vehicle drives using local sensors only, no cooperation, no network |
| **Scenario 2 — Cooperative V2X (no Konro)** | Vehicles cooperate through OMNeT++, fixed CPU allocation (no resource management) |
| **Scenario 3 — Cooperative V2X + Konro** | Vehicles cooperate through OMNeT++ AND Konro dynamically allocates CPU PUs |

### 2.2 Variables

| Variable | Values |
|----------|--------|
| Map | Town03, Town04, Town05 |
| Number of vehicles | 4, 6, 10 |
| Number of pedestrians | 4 (stationary or slow-walking, no sensors) |
| Network condition | Line-of-sight, occluded (behind buildings), degraded (packet loss via OMNeT++) |
| Cooperation | ON / OFF |
| Konro | ON / OFF |
| Fixed delta seconds | 0.05 s (20 Hz synchronous simulation) |
| Duration per run | 60 seconds |

### 2.3 Total runs

With 3 maps × 3 vehicle counts × 2 network conditions × 3 configurations = **54 runs**.
Each run should be repeated at least 3 times with different seeds for statistical validity
(target: 162 runs).

Pedestrians are detected by vehicle LiDAR/camera and shared through cooperative
perception (OMNeT++). They do not carry sensors themselves.

## 3. KPIs to Measure (from ENERGY_MODEL_PARAMETERS.md sources)

### 3.1 Perception KPIs

| KPI | Definition | Source |
|-----|------------|--------|
| Recall | TP / (TP + FN) | Standard detection metric |
| Precision | TP / (TP + FP) | Standard detection metric |
| F1 | 2 × (Precision × Recall) / (Precision + Recall) | Standard detection metric |
| Number of fused boxes | Count of cooperative boxes fused per frame | Internal metric |

### 3.2 Network KPIs (from OMNeT++)

| KPI | Definition | Unit | Source |
|-----|------------|------|--------|
| Latency | End-to-end packet delay | s | OMNeT++ CoPerceptionApp |
| Jitter | Variation in packet delay | s | OMNeT++ CoPerceptionApp |
| Packet loss | Dropped / total packets | % | OMNeT++ sequence number gap |
| Age of Information (AoI) | t - u(t), timestamp of last received box | s | KIT / arXiv V2X Collaborative Perception |

### 3.3 Resource KPIs (from Konro)

| KPI | Definition | Unit | Source |
|-----|------------|------|--------|
| CPU load | Utilization of allocated cores | % | Linux cgroups / Konro |
| PUs allocated | Number of CPU cores assigned by Konro | count | Konro cpuset |
| Deadline miss rate | Frames where inference exceeded deadline | % | Internal metric |

### 3.4 Energy KPIs (from energy_model.py)

| KPI | Definition | Unit | Source |
|-----|------------|------|--------|
| Instantaneous power | P_compute at frame t | W | Lin et al. ASPLOS 2018 |
| Cumulative energy | Sum of power × duration | J | Lin et al. ASPLOS 2018 |
| Power breakdown | idle / core / NPU / base vehicle | W | VehicleEnergyModel |

### 3.5 Safety KPIs (from UNECE R157 / NAVSIM)

| KPI | Definition | Unit | Threshold |
|-----|------------|------|-----------|
| Time-to-Collision (TTC) | Δd / Δv | s | AEB if < 1.5 s |
| Stopping distance | v₀·(t_react + t_lag) + v₀²/(2·μ·g) | m | UNECE R157 |
| Emergency brake count | Number of brake = 1.0 events | count | Internal metric |

### 3.6 Comfort KPIs

| KPI | Definition | Unit | Threshold |
|-----|------------|------|-----------|
| Kinematic jerk | da/dt = d³x/dt³ | m/s³ | Comfort ≤ 1.0, Urban max ≤ 2.5, Emergency > 5.0 |

## 4. File to Create and Modify

| File | Action | Description |
|------|--------|-------------|
| `tools/carla/analyze_results.py` | Create | Post-processing script: load JSON metrics, compute KPIs, generate charts |
| `tools/carla/run_experiment_matrix.py` | Create | Batch runner: executes multiple scenarios automatically |
| `docs/thesis/TASK-04_Evaluation_KPI.md` | Create | This specification document |
| `docs/thesis/PROJECT_LOG.md` | Update | Record Phase 4 start |
| `docs/thesis/FUTURE_FEATURES_BACKLOG.md` | Update | Mark PF-16 to PF-20 as implemented/in progress |

## 5. Output Files

| File | Description |
|------|-------------|
| `logs/carla_run/carlaloop_*.json` | Per-run metrics (already generated by run_closed_loop.py) |
| `logs/carla_run/analysis_*.json` | Aggregated KPIs per condition |
| `logs/carla_run/analysis_*.png` | Comparative charts (recall vs. stopping distance, energy vs. TTC, etc.) |

## 6. Success Criteria

| Criterion | Pass condition |
|-----------|----------------|
| Experiment matrix executed | All 54 runs (or subset) completed without crash |
| Konro value demonstrated | Scenario 3 shows better energy/quality trade-off than Scenario 2 |
| Safety maintained | No collisions due to resource starvation; emergency brake works in all conditions |
| Data repeatability | Runs with same seed produce identical results (determinism) |
| Charts generated | At least 4 comparative charts produced |

## 7. Generalization Path (Post-TASK-04)

After TASK-04 is validated, the same framework can support:
- Real CoPerception model inference (Phase 1.2 placeholder replacement)
- Building-aware OMNeT++ propagation (OMNET_LIMITATIONS_AND_ENHANCEMENTS.md)
- Different SoC configurations per vehicle (e.g., Ego = Thor, Partner = Orin)
- Docker containerization for reproducibility (PF-24)

---

*End of TASK-04_Evaluation_KPI.md*