# TASK-02 — OMNeT++ Network Injection (Phase 2)

**Project:** Master Thesis — CARLA + CoPerception + OMNeT++ + Konro Integration  
**Task ID:** TASK-02  
**Phase:** Phase 2 — OMNeT++ Network Simulation Injection  
**Status:** Specification (not yet implemented)  
**Language:** English  
**Owner:** Alberto  
**Created:** 2026-11-11

## 1. Objective

Replace the **in-memory cooperative registry** (direct bounding box sharing between CARLA vehicles) with a **realistic V2X network simulation** through OMNeT++ + INET (IEEE 802.11p). Vehicles no longer exchange data directly; they communicate through the network simulator that introduces:

- **Packet loss** based on distance, transmission power, receiver sensitivity, interference
- **Propagation delay** and jitter
- **Age of Information (AoI)** on received data

The ego vehicle makes driving decisions **exclusively** on data that arrives through OMNeT++ (delayed or lost), never on CARLA privileged ground-truth.

## 2. Architecture

### 2.1 Central bridge (Option 1)

A single `OMNetBridge` runs in the main Python process and manages one TCP connection to the OMNeT++ `NetworkManager` on port 5555. The bridge:

- Sends `move` commands to update each vehicle's position in the network simulator
- Sends `send` commands to request packet transmission between vehicle pairs
- Receives `received` notifications with `delay` and `deliver` fields
- Forwards network decisions to the corresponding `CooperativeAgent`

### 2.2 Communication protocol (JSON over TCP)

**CARLA → OMNeT++:**

| Command | JSON fields | Description |
|---------|-------------|-------------|
| `move` | `{"type": "move", "id": "<vid>", "x": ..., "y": ..., "z": ...}` | Update vehicle position in network simulator |
| `send` | `{"type": "send", "src": "<vid>", "dst": "<vid>", "size": <bytes>, "id": "<msg_id>"}` | Request packet transmission from src to dst |

**OMNeT++ → CARLA:**

| Command | JSON fields | Description |
|---------|-------------|-------------|
| `received` | `{"type": "received", "id": "<msg_id>", "delay": , "deliver": true/false}` | Notification of transmission outcome: delivered/lost + delay |

### 2.3 Per-tick flow (per vehicle)

At each `world.tick()`:

1. Read sensors (LiDAR + camera) → PyTorch tensors
2. Run CoPerception on local data → local bounding boxes
3. Send `move` to OMNeT++ with current vehicle position
4. Send `send` to OMNeT++ for each cooperative partner (feature map transmission request)
5. Read `received` responses from OMNeT++ (delay + deliver status)
6. Apply network effect:
   - `deliver=false` → packet lost: substitute with zeros or oldest buffered data
   - `delay > 0` → retrieve data from `frames_lag = round(delay * framerate)` frames ago from circular buffer
   - `delay ≈ 0` → use current data (live)
7. Fuse received boxes (local + delivered partner boxes)
8. Execute control (`run_step`): emergency brake, speed reduction, or normal driving
9. Apply control to CARLA vehicle

## 3. Files to create and modify

| File | Action | Description |
|------|--------|-------------|
| `tools/carla/omnet_bridge.py` | Create | Async TCP client for OMNeT++ communication (move/send commands, received notifications) |
| `tools/carla/cooperative_agent.py` | Modify | Replace in-memory `_cooperative_registry` with OMNeT++ bridge calls for packet exchange |
| `tools/carla/run_closed_loop.py` | Modify | Integrate OMNetBridge into simulation loop: send positions/packets, read network responses |
| `docs/thesis/TASK-02_OMNeT_Network_Injection.md` | Create | This specification file |
| `docs/thesis/PROJECT_LOG.md` | Update | Record start of Phase 2 |

## 4. Success criteria

| Criterion | Pass condition |
|-----------|----------------|
| OMNeT++ connection | Python bridge successfully connects to OMNeT++ server on port 5555 |
| Position sync | Vehicle positions are sent to OMNeT++ every tick via `move` commands |
| Packet exchange | Feature map transmissions are requested via `send` commands |
| Network response | `received` notifications are read with delay and deliver fields |
| Packet loss effect | Lost packets result in degraded perception (zeros or oldest buffer) |
| Delay effect | Delayed packets result in perceptual lag from circular buffer |
| No ground-truth | Driving decisions use only data delivered through OMNeT++ |

## 5. Dependencies (already available)

- OMNeT++ 6.0.3 compiled (`omnet_sim` binary at `omnet_sim/omnet_sim`)
- INET 4.5 at `/home/alberto/inet4.5/src`
- `NetworkManager.cc` already implements TCP server on port 5555, `move`/`send` command processing, `received` notifications
- Port 5555 already consistent between `omnetpp.ini` and client

---

*End of TASK-02_OMNeT_Network_Injection.md*