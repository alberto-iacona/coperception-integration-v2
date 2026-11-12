# OMNeT++ Network Model — Limitations and Planned Enhancements

**Project:** Master Thesis — CARLA + CoPerception + OMNeT++ + Konro Integration  
**Document Type:** Technical analysis / enhancement backlog  
**Language:** English  
**Created:** 2026-11-12  
**Status:** Active record

## 1. Purpose

This document records the current limitations of the OMNeT++ + INET network model used in Phase 2, and lists all possible enhancements to make the V2X network simulation more realistic (especially regarding environment-aware propagation, obstacles, and channel saturation). These items are **deferred** to future phases and should be revisited when the core closed-loop integration (CARLA + CoPerception + OMNeT++ + Konro) is stable.

---

## 2. Current Network Model (Phase 2 — implemented)

The OMNeT++ simulation uses **INET 4.5** with `Ieee80211ScalarRadioMedium`. Configuration in `omnetpp.ini`:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Radio medium | `Ieee80211ScalarRadioMedium` | Distance-based scalar propagation model |
| Background noise | -110 dBm | Thermal noise floor |
| Transmitter power | 20 mW | Per-vehicle Wi-Fi/802.11p radio |
| Receiver sensitivity | -85 dBm | Minimum signal for successful reception |
| Mobility | `PythonMobility` | Positions updated from CARLA via TCP `move` commands |
| Scheduler | `cRealTimeScheduler` | Real-time synchronization with wall clock |

### What IS physically simulated (realistic)
- **Path loss**: signal attenuation with distance (INET built-in)
- **Receiver sensitivity threshold**: packets below -85 dBm are not received
- **Background noise**: -110 dBm thermal noise
- **Interference**: simultaneous transmissions from multiple vehicles can corrupt each other
- **CSMA/CA contention**: vehicles wait for a clear channel before transmitting (backoff, collisions)
- **End-to-end delay**: measured by `CoPerceptionApp` as `(arrival_time - creation_time)`
- **Packet loss detection**: Python detects gaps in sequence numbers when ACKs are missing

### What is NOT simulated (limitations)
| Limitation | Impact | Severity |
|------------|--------|----------|
| **No obstacle / building shadowing** | `Ieee80211ScalarRadioMedium` is distance-only. Two vehicles separated by a building are treated as line-of-sight if within range. | High (for urban scenarios) |
| **No building / map-aware propagation** | No building footprints, no diffraction, no reflection from urban structures | High |
| **No explicit channel saturation model** | Degradation comes only from native 802.11p CSMA/CA contention, not from a configurable urban saturation threshold | Medium |
| **No fading / multipath** | No Rayleigh/Rician fading, no urban multipath effects | Medium |
| **No weather / atmospheric effects** | Rain, fog, humidity not modeled | Low |

---

## 3. Possible Enhancements for Environment-Aware Transmission

The following additions would make the network model dependent on the **physical scene** (buildings, obstacles, urban canyon), not just distance.

### 3.1 Building / Obstacle Shadowing (High priority)

**Goal:** Vehicles separated by buildings should experience additional attenuation even if within radio range.

**Possible approaches:**

| Approach | Description | Effort | Files to modify |
|----------|-------------|--------|-----------------|
| `LogNormalShadowing` | Replace `Ieee80211ScalarRadioMedium` with INET's `LogNormalShadowing` or `TwoRayGround` for more realistic urban fading | Medium | `omnetpp.ini`, `CoSim.ned` |
| Obstacle loss model | Configure `obstacleLoss` parameter in INET to attenuate signals passing through building polygons | Medium | `omnetpp.ini` |
| Building footprint import | Import OpenStreetMap or CARLA road network into OMNeT++ as building polygons for shadowing calculation | High | `CoSim.ned`, new NED file |
| Ray-tracing attenuation | Custom module that checks line-of-sight between vehicles against CARLA building geometry and applies extra dB loss | High | New C++ module in `omnet_sim/` |

### 3.2 Channel Saturation / Urban Congestion (Medium priority)

**Goal:** Model explicit degradation when many vehicles transmit in a dense urban area.

| Approach | Description | Effort |
|----------|-------------|--------|
| Increased contention window | Dynamically increase CWmin/CWmax as vehicle density grows | Medium |
| Packet drop probability by density | Add a drop probability function of `numHosts` and local vehicle density | Medium |
| Time-varying interference map | Pre-compute interference zones from CARLA map and feed to OMNeT++ | High |

### 3.3 Fading and Multipath (Medium priority)

| Approach | Description | Effort |
|----------|-------------|--------|
| Rayleigh fading | Add small-scale fading to the radio medium | Low-Medium |
| Shadowing correlation | Correlate shadowing over time/space (vehicles close together see similar shadowing) | Medium |

### 3.4 Weather and Atmospheric Effects (Low priority)

| Approach | Description | Effort |
|----------|-------------|--------|
| Rain attenuation | Additional dB loss during rain events in CARLA | Low |
| Temperature / humidity | Minor effect on radio propagation | Low |

---

## 4. Integration with CARLA Scene

To make the network **scene-aware**, the following data flow could be added:

```
CARLA (building polygons, road network)
    → Export building footprints (OSM or CARLA static geometry)
    → OMNeT++ (building polygon modules for shadowing)
    → Radio propagation with obstacle loss
    → Packet loss/delay depends on physical scene, not just distance
```

This would allow experiments such as:
- "Ego vehicle behind a building loses cooperative data from partner" (tests closed-loop safety degradation)
- "Urban canyon on Town04 main street causes 40% packet loss between adjacent vehicles"

---

## 5. When to Implement

These enhancements are **deferred** because:
1. The current distance-based model is sufficient to validate the closed-loop architecture (Fases 1-2).
2. Adding building-aware propagation requires significant OMNeT++ C++ development and validation.
3. The thesis's novel contribution (Konro + closed-loop perception) does not strictly require building-aware radio; it requires a working network with realistic loss/delay.

**Recommended phase:** Phase 3 or Phase 4, after Konro integration is stable and the core experiments are running.

---

## 6. References

- INET Framework documentation: `Ieee80211ScalarRadioMedium`, `LogNormalShadowing`, `TwoRayGround`
- CARLA static geometry / OpenStreetMap export for building footprints
- IEEE 802.11p / ITS-G5 urban propagation models

---

*End of OMNET_LIMITATIONS_AND_ENHANCEMENTS.md*