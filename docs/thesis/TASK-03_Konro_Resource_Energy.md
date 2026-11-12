# TASK-03 — Konro Resource Management & Energy Modeling (Phase 3)

**Project:** Master Thesis — CARLA + CoPerception + OMNeT++ + Konro Integration  
**Task ID:** TASK-03  
**Phase:** Phase 3 — Energy/Resource Modeling & Konro Dynamic Allocation  
**Status:** Specification (not yet implemented)  
**Language:** English  
**Owner:** Alberto  
**Created:** 2026-11-12

## 1. Objective

Integrate **Konro** as a run-time resource manager that dynamically allocates CPU Processing Units (PUs via Linux cgroups/cpuset) to the perception processes of CARLA vehicles. Allocation decisions are based on feedback from the quality of perception (proxy metric: recall × false-positive penalty) computed locally and transmitted to Konro via HTTP. The system balances perception quality against on-board energy consumption, respecting automotive safety constraints (ISO 26262 ASIL-D, UNECE R157 ALKS).

This phase transitions the project from a "theoretical" design to a **simulative reality**: each vehicle is modeled with a realistic on-board computer (SoC) with configurable idle power, per-core power, NPU power, and base vehicle power (safety functions that must never be compromised).

## 2. Energy Model (on-board computer)

### 3.1 Formal model

The computational power consumption of the on-board computer follows the model from Lin et al. (ASPLOS 2018) and DLR component-level analysis:

```
P_compute(t) = P_idle + Σ(P_core(f_i) · U_core,i(t)) + P_NPU(t) + P_base_vehicle
```

where:
- **P_idle**: static board/memory power (typically 15%–25% of TDP)
- **P_core(f_i)**: dynamic CMOS power per core, proportional to C_eff · V_dd² · f_i
- **U_core,i(t)**: utilization of core i in [0, 1], controlled via Linux cgroups cpuset by Konro
- **P_NPU(t)**: accelerator (NPU/GPU) power during CoPerception inference
- **P_base_vehicle**: power of base vehicle functions (braking, steering, critical sensors, V2X) that must never be compromised

### 3.2 Default parameter values (derived from bibliographic research)

These values are **parametric defaults** derived from the sources in `docs/thesis/ENERGY_MODEL_PARAMETERS.md`. They must be validated/adapted to the specific hardware chosen for the thesis.

| Parameter | Default Value | Unit | Rationale / Source |
|-----------|---------------|------|--------------------|
| P_idle (board) | 20 | W | ~15–25% of 100–150 W typical SoC TDP |
| P_core (per core, 100%) | 2.0 | W | Derived from SoC TDP ranges (Orin 15–60 W / 12 cores) |
| P_NPU (inference active) | 25 | W | Derived from PointPillars active power ~23.8 W |
| P_base_vehicle (safety functions) | 100 | W | Conservative estimate for braking, steering, sensors, V2X |
| Total CPU cores available | 8 | count | Representative of mid-range automotive SoC |
| Baseline PUs (minimum for safety) | 2 | count | Never go below this; ensures ASIL-D safety functions |

### 3.3 Energy logging

For each simulation frame, the system logs:
- Instantaneous power (W)
- Cumulative energy (J)
- PUs allocated by Konro
- CPU utilization estimate
- NPU active flag

These metrics enable Pareto analysis: perception quality (recall) vs. energy consumption (Joule).

---

## 3. Konro Integration Logic

### 3.1 Feedback loop

```
Percezione (CoPerception) → Proxy quality (recall × FP penalty, [0-1])
                                    │
                                    ▼
                    KonroBridge sends feedback [0-200] to Konro (HTTP POST localhost:8080)
                                    │
                                    ▼
                    Konro decides: allocate more cores or remove cores
                                    │
                                    ▼
                    Linux cgroups cpuset: assigns/removes CPUs to the process
                                    │
                                    ▼
                    More PUs = faster inference = less frame lag
                                    │
                                    ▼
                    If radio is a bottleneck (timeout/blackout): Konro returns to baseline_pus (1 PU)
                                    │
                                    ▼
                    Energy is saved when the network does not need extra compute
```

### 3.2 Konro thresholds (from HEAPLab design)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| kLowThreshold | 160 | Feedback < 160 → add cores |
| kHighThreshold | 180 | Feedback > 180 → remove cores |
| kGiveUpTicks | 5 | After 5 consecutive low-feedback cycles, Konro concludes the bottleneck is radio, not CPU, and returns to baseline_pus |
| baseline_pus | 1 | Absolute minimum if radio is the bottleneck |

### 3.3 Safety constraints respected

| Constraint | Implementation |
|------------|----------------|
| ASIL-D / base functions never compromised | `baseline_pus = 2` always guaranteed; Konro never goes below this |
| UNECE R157 Minimum Risk Manoeuvre (MRM) | On radio blackout/timeout, Konro returns to baseline_pus and vehicle brakes/stops safely |
| TTC < 1.5 s = emergency brake | Already implemented in Phase 2; Konro never disables this |
| Jerk containment | Gradual speed reduction before emergency braking |

---

## 4. File to Create and Modify

| File | Action | Description |
|------|--------|-------------|
| `tools/carla/energy_model.py` | Create | On-board vehicle energy model with configurable parameters |
| `tools/carla/konro_integration.py` | Create | Konro integration: PID registration, feedback sending, PU reading |
| `tools/carla/run_closed_loop.py` | Modify | Integrate Konro into simulation loop + energy logging |
| `docs/thesis/PROJECT_LOG.md` | Update | Record Phase 3 start |
| `docs/thesis/FUTURE_FEATURES_BACKLOG.md` | Update | Add new ideas if they emerge during development |

---

## 5. Success Criteria

| Criterion | Pass condition |
|-----------|----------------|
| Konro connection | Python process registers with Konro on localhost:8080 |
| Feedback loop | Proxy metric is computed and sent to Konro every feedback_interval frames |
| PU allocation | Konro allocates/removes PUs via cgroups based on feedback |
| Energy model | Power and energy are logged per frame with correct breakdown |
| Safety baseline | Process never drops below baseline_pus (2 PUs) even during radio blackout |
| Closed-loop reaction | If perception degrades due to network loss, vehicle physically reacts (brake / TTC margin) |

---

## 6. Generalization Path (Post-TASK-03)

After TASK-03 is validated, the same code must support:
- N vehicles each with independent Konro management
- Different SoC configurations per vehicle (e.g., Ego has Thor, partner has Orin)
- Energy-aware scenario matrix (different target speeds, network conditions, Konro on/off)

---

*End of TASK-03_Konro_Resource_Energy.md*