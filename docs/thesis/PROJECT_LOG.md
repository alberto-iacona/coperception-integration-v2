# Project Log — CARLA + CoPerception + OMNeT++ + Konro Integration

**Project:** Master Thesis — Closed-loop network-aware cooperative perception with run-time resource management  
**Start date:** 2026-11-11  
**Author:** Alberto  
**Language:** Documentation in English

## Overview

This file records the chronological progress of the thesis project, including major milestones, decisions, problems encountered, and resolutions. Each entry is timestamped and linked to specific tasks or files.

## Timeline

### 2026-11-11 — Project initialization
- Repository analyzed: `/home/alberto/coperception-Integration/`
- Components already installed and verified:
  - CoPerception SDK (editable install in venv)
  - OMNeT++ 6.0.3 with INET 4.5 (`/home/alberto/omnetpp-6.0.3`, `/home/alberto/inet4.5`)
  - Konro resource manager (compiled binary at `konro/build/rm/konro`)
  - CARLA 0.9.15 server (`/home/alberto/carla/`, launcher `CarlaUE4.sh`) + Python client API
  - Python virtual environment at `venv/` (torch 2.5.1+cu121, opencv-python 4.10, mmcv, etc.)
- `start_up_procedure.txt` corrected: conda → venv, repo path `~/coperception-Integration/`, INET path `/home/alberto/inet4.5/src`
- `start_up_procedure.txt` and `Project.pdf` added to `.gitignore` and removed from git tracking
- Dataset V2X-Sim and checkpoints NOT yet downloaded (not required for CARLA integration)
- Design document (JSON-based architecture) reviewed: modular phases, closed-loop architecture, hard constraints confirmed

### 2026-11-11 — Documentation setup
- Created `docs/thesis/` directory
- Created PROJECT_LOG.md (this file)
- Created TASK-01_CARLA_CoPerception.md (task specification for Phase 1)
- Created ARCHITECTURE_DECISIONS.md (architecture decisions registry)
- Created ISSUES_AND_RESOLUTIONS.md (problem tracking)

### 2026-11-11 — TASK-01 code development (commit 479ec7f)
- Created branch `feature/phase-1-carla-integration` from `project_deliver`
- Implemented `tools/carla/scenario_setup.py`: CARLA connection, map loading,
  synchronous mode, parametrized vehicle/sensor spawning
- Implemented `tools/carla/sensor_adapter.py`: raw CARLA LiDAR/camera buffers
  -> PyTorch tensors for CoPerception
- Implemented `tools/carla/cooperative_agent.py`: BehaviorAgent subclass using
  CoPerception fused 3D bounding boxes for hazard detection (no privileged
  CARLA ground-truth)
- Implemented `tools/carla/run_closed_loop.py`: main orchestration script with
  metrics logging to JSON
- Created `tools/carla/__init__.py`
- All 5 Python files pass `py_compile` syntax check
- Commit 479ec7f created locally (11 files, +1649/-176 lines)
- **PENDING**: `git push origin feature/phase-1-carla-integration` blocked by
  missing GitHub credentials (username/token not configured in this session)

## Status

**Current phase:** Phase 1 — CARLA + CoPerception closed-loop integration  
**Current task:** TASK-01 — Code implemented and committed locally  
**Next action:** Configure GitHub credentials and push, OR await user instruction

## Open questions / blockers

- None at this time

---

*End of PROJECT_LOG.md*