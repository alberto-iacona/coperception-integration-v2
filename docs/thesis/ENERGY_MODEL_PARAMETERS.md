# Energy Model Parameters and Sources — Konro Resource Management (Phase 3)

**Project:** Master Thesis — CARLA + CoPerception + OMNeT++ + Konro Integration  
**Document Type:** Technical reference / energy model parameterization  
**Language:** English  
**Created:** 2026-11-12  
**Status:** Active record

## 1. Purpose

This document collects all hardware, energy, safety, and KPI parameters gathered from bibliographic research for the Phase 3 (Konro Resource Management & Energy Modeling). All values are traceable to their sources (manufacturer datasheets, academic literature, regulatory standards). Values marked as **estimates** must be validated with real measurements before being used in final thesis experiments.

---

## 2. On-Board SoC Parameters (Tema 1)

| SoC / Platform | CPU Cores | AI Accelerator | AI Performance | TDP / Power Range | Idle Power | Memory | Source |
|----------------|-----------|----------------|----------------|-------------------|------------|--------|--------|
| NVIDIA DRIVE AGX Thor | 14 × ARM Neoverse V3AE | Blackwell GPU (2560 CUDA + 96 Tensor Core 5th gen) + PVA | 1,000 INT8 TOPS / 2,000 FP4 TFLOPS | Developer Kit 350 W; SoC configurable 15 W – 120 W | Not officially reported | 64/128 GB LPDDR5X, 273 GB/s | [NVIDIA DRIVE AGX](https://developer.nvidia.com/drive/agx) |
| NVIDIA DRIVE AGX Orin | 12 × ARM Cortex-A78AE | Ampere GPU (2048 CUDA + 64 Tensor Core) + 2 × DLA | 254 INT8 TOPS (167 GPU + 87 DLA) | Developer Kit 200 W; SoC 15 W – 60 W | Not officially reported (~15 W low-power estimate) | 32/64 GB LPDDR5, 205 GB/s | [NVIDIA DRIVE AGX Orin](https://developer.nvidia.com/drive/agx) |
| Tesla HW4 (AI4 / FSD Computer 2) | 20 × ARM per chip (dual-SoC redundant) | 3 × NPU per chip at 2.2 GHz | Estimate ~50–140 TOPS per node (not officially declared) | Estimate ~100–160 W per dual-SoC board (~72 W for HW3) | Not officially reported | 16 GB GDDR6 / LPDDR5, 224–384 GB/s | [Wikipedia Tesla Autopilot hardware](https://en.wikipedia.org/wiki/Tesla_Autopilot_hardware) |
| Qualcomm Snapdragon Ride (QAM9000P) | Multi-core ARM (Kryo / Cortex-A78AE) | Dedicated ML/Vision accelerators + Adreno GPU + Safety Manager ASIL-D | Scalable 10 TOPS (ADAS camera) up to >700 TOPS (full autonomy; reference config 200 TOPS) | < 5 W (base ADAS) to ~65–130 W (multi-SoC high performance) | Not officially reported | LPDDR5 (capacity/bandwidth not publicly declared) | [Qualcomm Snapdragon Ride](https://docs.qualcomm.com/bundle/publicresource/topics/80-63195-3/snapdragon_ride_platform.html?product=1601111740010420) |
| Mobileye EyeQ6 High | 8 core CPU (32 threads) | Proprietary VMP/PMA/MPC accelerators | 34 INT8 TOPS | TDP 33 W | Not officially reported | LPDDR5, 7 nm FinFET | [Mobileye CES 2022](https://www.mobileye.com/news/mobileye-ces-2022-tech-news/) |
| Mobileye EyeQ Ultra | 12 core CPU (RISC-V/ARM) | Proprietary accelerators | 176 INT8 TOPS | TDP < 100 W | Not officially reported | LPDDR5X, 5 nm | [Mobileye CES 2022](https://www.mobileye.com/news/mobileye-ces-2022-tech-news/) |

**Notes:**
- NVIDIA DRIVE AGX Orin includes a Safety Island with 4 lock-step ARM Cortex-R52 pairs certified ISO 26262 ASIL-D.
- Tesla HW4 uses dual-SoC redundancy for fault tolerance; Tesla does not publish official datasheets.

---

## 3. Computational Energy Model Parameters (Tema 2)

### Formal model

The standard reference model for embedded computing power in autonomous driving:

```
P_compute(t) = P_idle + Σ(P_core(f_i) · U_core,i(t)) + P_acc(U_NPU/GPU(t))
```

where:
- **P_idle**: static board/memory power (typically 15%–25% of TDP)
- **P_core(f)**: dynamic CMOS power per core, proportional to C_eff · V_dd² · f
- **U_core,i(t)**: utilization of core i in [0, 1], controlled via Linux cgroups cpuset by Konro
- **P_acc**: accelerator power (NPU/GPU), function of accelerator utilization

### Parameter values from literature

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| P_idle share of TDP | 15% – 25% | % | Lin et al., ASPLOS 2018 |
| PointPillars baseline latency | 36.1 ms | per frame (~27.7 FPS) | Balasubramaniam et al., UPAQ 2024/2025 |
| PointPillars baseline energy | 0.86 J | per inference (~23.8 W active) | Balasubramaniam et al., UPAQ 2024/2025 |
| PointPillars optimized energy | 0.472 J | per inference (quantized UPAQ) | Balasubramaniam et al., UPAQ 2024/2025 |
| SMOKE baseline energy | 29.2 J | per inference (68.45 ms) | Balasubramaniam et al., UPAQ 2024/2025 |
| SMOKE optimized energy | 15.62 J | per inference | Balasubramaniam et al., UPAQ 2024/2025 |
| Latency with 1 CPU core | > 100 ms | deadline violation at 10 Hz | Michigan Tech University thesis |
| Latency with 4–5 CPU cores | < 25 ms | meets 10 Hz deadline | Michigan Tech University thesis |
| CPU energy throttled (1 core) | ~0.013 Wh/km | | Michigan Tech University thesis |
| CPU energy full load (4–5 cores) | ~0.116 Wh/km | | Michigan Tech University thesis |
| Auxiliary system total (compute + sensors) | 250 W – 1,000 W | depending on architecture | DLR component-level analysis |
| Computer share of auxiliary power | 50% – 70% | | DLR component-level analysis |
| Sensors share of auxiliary power | 20% – 35% | | DLR component-level analysis |

**Notes:**
- PointPillars / SMOKE values demonstrate that accelerating inference via parallelism or compression reduces total energy per frame (E = P · Δt) provided instantaneous power increase is lower than the time gain.
- CPU bottleneck in cooperative pipelines is geometric coordinate transformation and temporal alignment of tensors received via network, not convolution (which runs on GPU/NPU).

---

## 4. Safety and Regulatory Parameters (Tema 4)

### ISO 26262 ASIL levels

| Level | Meaning | Hardware diagnostic coverage | PMHF (random hardware failure) |
|-------|---------|------------------------------|-------------------------------|
| QM | Quality Management | — | — |
| ASIL A | Lowest automotive criticality | — | — |
| ASIL B | Moderate criticality | — | — |
| ASIL C | High criticality | — | — |
| ASIL-D | Maximum criticality (S3, E4, C3) | ≥ 99% | < 10⁻⁸ h⁻¹ (10 FIT) |

**Source:** ISO 26262:2018 — Road vehicles — Functional safety

### UNECE R157 (ALKS — Automated Lane Keeping Systems)

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| Original max operating speed | 60 | km/h | UNECE R157 (2021) |
| Extended max operating speed (Amendment 4, 2023) | 130 | km/h | UNECE R157 Amendment 4 |
| Max lateral acceleration in emergency manoeuvre | 1.0 | m/s² | UNECE R157 |
| Imminent collision risk: not avoidable with deceleration below | 5 | m/s² | UNECE R157 |
| Required action on sensor/communication failure | Minimum Risk Manoeuvre (MRM) — bring vehicle to safe stop | — | UNECE R157 |

**Key principle for thesis:** Loss of cooperative V2X channel must **never** compromise basic longitudinal and lateral control functions. Minimum resources must remain inviolable for local driving.

---

## 5. Safety and Comfort KPIs (Tema 5)

| KPI | Definition | Safe / Normal | Alert (FCW) | Emergency (AEB) | Source |
|-----|------------|---------------|-------------|-----------------|--------|
| Time-to-Collision (TTC) | Δd / Δv | > 4.0 s | 2.0 – 3.0 s | < 1.5 s (0.8–1.2 s urban) | NAVSIM / SAE engineering reference |
| Stopping distance | d_stop = v₀·(t_react + t_lag_perception) + v₀²/(2·μ·g) | — | — | — | Standard vehicle dynamics / UNECE R157 |
| Kinematic jerk | j(t) = da/dt = d³x/dt³ | 0.5 – 1.0 m/s³ (comfort) | ≤ 2.0–2.5 m/s³ (urban max) | > 5.0–8.0 m/s³ (emergency/phantom braking) | Cao et al., NAVSIM v2 / ISO 2631 |
| Age of Information (AoI) | Δ(t) = t − u(t) | < 100 ms (useful for fusion) | — | > 300 ms (stale, unusable for active control) | KIT / arXiv V2X Collaborative Perception |

**Stopping distance example:** At 50 km/h (13.89 m/s) on dry asphalt (μ = 0.8), a perception lag of 200 ms adds 2.78 m to total stopping distance (~15.1 m total).

**AoI note:** In cooperative perception, AoI unifies network transmission delay (OMNeT++) and processing delay regulated by Konro: AoI = T_network + T_queue + T_inference.

---

## 6. Vehicle Energy and Autonomy Impact (Tema 6)

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| BEV average energy consumption | 191 | Wh/km | EV Database (WLTP / real-world combined) |
| Tesla Model 3 RWD | 130 – 142 | Wh/km | EV Database |
| SUV average (Model Y / ID.4) | 158 – 175 | Wh/km | EV Database |
| Full autonomy system (compute + sensors) | 500 – 1,000 | W | DLR / MDPI Energies review |
| Additional energy at 30 km/h urban | 16.7 – 33.3 | Wh/km | DLR / MDPI Energies review |
| Incidence on total consumption (urban) | ~10% – 20% | % | DLR / MDPI Energies review |
| Incidence on total consumption (highway) | ~3% – 6% | % | DLR / MDPI Energies review |
| Estimated autonomy reduction (AV vs non-AV) | 9% – 15% | % | DLR / MDPI Energies review |
| Perception energy saving via adaptive scheduling | 10% – 58% | % | DLR / MDPI Energies review |

**Note:** At urban speeds (30–50 km/h), Konro's energy optimization has maximum impact on vehicle range. At highway speeds (110–130 km/h), aerodynamic drag dominates and the on-board computer incidence drops.

---

## 7. Default Values for Simulation (Phase 3)

The following values are **parametric defaults** for the CARLA simulation. They are derived from the sources above but must be validated/adapted to the specific hardware chosen for the thesis.

| Parameter | Default Value | Unit | Rationale / Source |
|-----------|---------------|------|--------------------|
| P_idle (board) | 20 | W | ~15–25% of 100–150 W typical SoC TDP |
| P_core (per core, 100%) | 2.0 | W | Derived from SoC TDP ranges (Orin 15–60 W / 12 cores) |
| P_NPU (inference active) | 25 | W | Derived from PointPillars active power ~23.8 W |
| P_base_vehicle (safety functions) | 100 | W | Conservative estimate for braking, steering, sensors, V2X |
| Total CPU cores available | 8 | count | Representative of mid-range automotive SoC |
| Baseline PUs (minimum for safety) | 2 | count | Never go below this; ensures ASIL-D safety functions |
| Konro kLowThreshold | 160 | feedback units | Feedback < 160 → add cores |
| Konro kHighThreshold | 180 | feedback units | Feedback > 180 → remove cores |
| Konro kGiveUpTicks | 5 | consecutive ticks | After 5 low-feedback cycles, conclude radio bottleneck, return to 1 PU |
| Konro baseline_pus | 1 | PU | Absolute minimum if radio is the bottleneck |
| Dataset framerate | 5.0 | Hz | Matches V2X-Sim / CARLA synchronous mode default |
| Max buffer seconds | 2.0 | s | Buffer for delayed packets |
| TTC threshold | 3.0 | s | Hazard detection threshold |
| TTC emergency brake fraction | 0.5 | × threshold | Emergency brake when TTC < 50% of threshold |

---

## 8. Collected Sources (Numbered List)

### Theme 1 — SoC for Autonomous Driving

1. NVIDIA Corporation — *DRIVE AGX Autonomous Vehicle Development Platform*: https://developer.nvidia.com/drive/agx
2. NVIDIA Developer Blog — *Accelerate Autonomous Vehicle Development with the NVIDIA DRIVE AGX Thor Developer Kit*: https://developer.nvidia.com/blog/accelerate-autonomous-vehicle-development-with-the-nvidia-drive-agx-thor-developer-kit/
3. NVIDIA DRIVE AGX Orin Platform Specifications: https://www.scribd.com/document/1014704342/Nvidia-Drive-Agx-Orin-Platform-for-Developers
4. Wikipedia — *Tesla Autopilot hardware*: https://en.wikipedia.org/wiki/Tesla_Autopilot_hardware
5. Comparative hardware analysis — *Tesla HW 4 vs HW 3*: https://www.yeslak.com/blogs/tesla-guide/tesla-hardware-4-vs-hardware-3
6. Qualcomm Technologies — *Snapdragon Ride Platform Documentation*: https://docs.qualcomm.com/bundle/publicresource/topics/80-63195-3/snapdragon_ride_platform.html?product=1601111740010420
7. Qualcomm Newsroom — *Expansion of Snapdragon Ride Platform Portfolio*: https://www.qualcomm.com/news/releases/2021/01/qualcomm-announces-expansion-scalable-snapdragon-ride-platform-portfolio
8. Mobileye Newsroom — *New Mobileye EyeQ Ultra will Enable Consumer AVs*: https://www.mobileye.com/news/mobileye-ces-2022-tech-news/
9. Wikipedia — *Mobileye EyeQ Comparison*: https://en.wikipedia.org/wiki/Mobileye

### Theme 2 — Computational Energy Model

10. M. Senzeybek, G. Möring-Martinez (DLR) — *Understanding Energy Consumption in Autonomous Vehicles: A Component-Level Analysis*: https://elib.dlr.de/226547/3/TRA_108_poster_Energy%20Consumption%20in%20AVs.pdf
11. S.-C. Lin et al. — *Computing Systems for Autonomous Driving: State-of-the-Art and Challenges* (arXiv:2009.14349): https://arxiv.org/html/2009.14349v3
12. A. Balasubramaniam et al. — *UPAQ: A Framework for Real-Time and Energy-Efficient 3D Object Detection in Autonomous Vehicles*: https://www.researchgate.net/figure/Comparison-of-reduction-in-energy-usage-achieved-in-a-PointPillars-and-b-SMOKE-models_fig4_387862557
13. NSF PAR — *The Architectural Implications of Multi-modal Detection Models for Autonomous Driving*: https://par.nsf.gov/servlets/purl/10510976
14. Michigan Technological University — *Using Automotive Lidar to Reduce the Energy Consumption of an Autonomous Vehicle*: https://digitalcommons.mtu.edu/cgi/viewcontent.cgi?article=3260&context=etdr

### Theme 3 — Konro Resource Manager and Linux cgroups

15. HEAPLab (Politecnico di Milano) — *Konro Official GitHub Repository*: https://github.com/HEAPLab/konro
16. G. E. Lupi — *KONRO: An Adaptive Approach to Computing Resource Management* (M.Sc. Thesis, Politecnico di Milano): https://www.politesi.polimi.it/retrieve/ee656385-92e5-49e9-a81f-7ba3c225bd72/2023_05_Lupi_Executive%20Summary_02.pdf
17. Linux Kernel Source Tree — *cgroup/cpuset.c*: https://github.com/torvalds/linux/blob/master/kernel/cgroup/cpuset.c

### Theme 4 — Regulations and Automotive Safety Standards

18. Wikipedia — *Automotive Safety Integrity Level (ISO 26262)*: https://en.wikipedia.org/wiki/Automotive_safety_integrity_level
19. Agnile Technologies — *ISO 26262 ASIL Levels Explained: A to D*: https://agnile.com/blog/iso-26262-asil-levels
20. UNECE — *UN Regulation No. 157 — Automated Lane Keeping Systems (ALKS)*: https://unece.org/transport/documents/2021/03/standards/un-regulation-no-157-automated-lane-keeping-systems-alks
21. UNECE — *UN Regulation No. 157 Amendment 4 (130 km/h)*: https://unece.org/sites/default/files/2024-07/R157am4e_1.pdf

### Theme 5 — Safety and Comfort KPIs

22. arXiv — *Toward a Holistic Multi-Criteria Trajectory Evaluation Framework for Autonomous Driving* (arXiv:2509.01291): https://arxiv.org/pdf/2509.01291
23. W. Cao et al. — *Pseudo-Simulation for Autonomous Driving (NAVSIM v2)*: https://vveicao.github.io/projects/NavsimV2/Cao2025_supp.pdf
24. Karlsruhe Institute of Technology (KIT) — *Cooperative Perception in Connected Vehicles*: https://publikationen.bibliothek.kit.edu/1000197113
25. arXiv — *Minimizing AoI for Collaborative Perception through Online Learning* (arXiv:2607.20967): https://arxiv.org/html/2607.20967v1
26. University of Luxembourg (ORBilu) — *Cooperative Perception Using V2X Communications: An Experimental Study*: https://orbilu.uni.lu/bitstream/10993/63217/1/Cooperative_Perception_Using_V2X_Communications_An_Experimental_Study.pdf

### Theme 6 — Electric Vehicle Energy Consumption and Autonomy Impact

27. EV Database — *Energy consumption of full electric vehicles cheatsheet*: https://ev-database.org/cheatsheet/energy-consumption-electric-car
28. ResearchGate — *Data and Energy Impacts of Intelligent Transportation — A Review*: https://www.researchgate.net/publication/381495195_Data_and_Energy_Impacts_of_Intelligent_Transportation-A_Review
29. ResearchGate — *Predictive modeling of energy consumption and greenhouse gas emissions from autonomous electric vehicle operations*: https://www.researchgate.net/publication/334598918_Predictive_modeling_of_energy_consumption_and_greenhouse_gas_emissions_from_autonomous_electric_vehicle_operations

---

*End of ENERGY_MODEL_PARAMETERS.md*