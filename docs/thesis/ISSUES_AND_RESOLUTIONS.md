# Issues and Resolutions — CARLA + CoPerception + OMNeT++ + Konro

**Project:** Master Thesis — Closed-loop network-aware cooperative perception with run-time resource management  
**Language:** English  
**Created:** 2026-11-11  
**Status:** Active record

## Purpose

This file tracks every problem encountered during development, its root cause, the solution applied, and any relevant file changes. It serves as the troubleshooting reference for the thesis.

---

## Issue #001 — Conda environment referenced in start_up_procedure.txt not available

**Date:** 2026-11-11  
**Severity:** High (blocks legacy procedure)  
**Description:** `start_up_procedure.txt` used `conda activate coperception`, but conda is not installed on this Ubuntu system. Only a Python virtual environment (`venv/`) exists.  
**Root cause:** Procedure was written for a different machine where conda was available.  
**Resolution:** Replaced all `conda activate coperception` with `source ~/coperception-Integration/venv/bin/activate`.  
**Files changed:** `start_up_procedure.txt`  
**Status:** Resolved

---

## Issue #002 — Repository path mismatch in start_up_procedure.txt

**Date:** 2026-11-11  
**Severity:** High (blocks legacy procedure)  
**Description:** Procedure referenced `~/coperception/coperception-Integration/`, but the repository is located at `~/coperception-Integration/`.  
**Root cause:** Paths copied from a previous machine with a different directory structure.  
**Resolution:** Replaced all occurrences of `~/coperception/coperception-Integration/` with `~/coperception-Integration/`.  
**Files changed:** `start_up_procedure.txt`  
**Status:** Resolved

---

## Issue #003 — INET framework path mismatch in OMNeT++ launch command

**Date:** 2026-11-11  
**Severity:** High (blocks OMNeT++ startup)  
**Description:** Launch command used `-n ".:../../../inet/src"`, resolving to `~/inet/src`, but INET 4.5 is installed at `/home/alberto/inet4.5/src`.  
**Root cause:** Relative path assumed a symlink `~/inet` that does not exist.  
**Resolution:** Changed to `-n ".:/home/alberto/inet4.5/src"`.  
**Files changed:** `start_up_procedure.txt`  
**Status:** Resolved

---

## Issue #004 — start_up_procedure.txt and Project.pdf tracked by git

**Date:** 2026-11-11  
**Severity:** Medium (repository hygiene)  
**Description:** Both files contain machine-specific paths and should not be versioned.  
**Root cause:** Files were added to git before being added to `.gitignore`.  
**Resolution:** Added both files to `.gitignore`; removed `start_up_procedure.txt` from git tracking with `git rm --cached` (file kept on disk). `Project.pdf` was already untracked.  
**Files changed:** `.gitignore`, git index  
**Status:** Resolved

---

## Issue #005 — V2X-Sim dataset and checkpoints missing

**Date:** 2026-11-11  
**Severity:** Medium (blocks dataset-based testing, not CARLA integration)  
**Description:** Directory `data/` does not exist; no `.pth` checkpoint files are present.  
**Root cause:** Fresh Ubuntu installation; datasets were on other PCs.  
**Resolution:** Not required for Phase 1 (CARLA + CoPerception uses live sensors, not datasets). Deferred: dataset download only if regression testing on V2X-Sim is needed later.  
**Files changed:** None  
**Status:** Deferred (not blocking)

---

## Issue #006 — nuscenes_devkit missing from venv

**Date:** 2026-11-11  
**Severity:** Low (not required for CARLA Phase 1)  
**Description:** `requirements.txt` lists `nuscenes-devkit==1.0.9`, but it is not installed in the venv.  
**Root cause:** Incomplete pip install on fresh machine.  
**Resolution:** Not blocking for Phase 1. Install only if nuScenes dataset support is needed.  
**Files changed:** None  
**Status:** Deferred

---

## Issue #007 — CARLA client/server version mismatch (segfault)

**Date:** 2026-11-11  
**Severity:** High (blocks CARLA connection)  
**Description:** `pip install carla` installed version 0.9.16, but the CARLA
server is version 0.9.15. Connecting caused a segmentation fault.  
**Root cause:** PyPI `carla` package defaulted to 0.9.16, incompatible with
CARLA 0.9.15 server.  
**Resolution:** `pip install --force-reinstall --no-deps "carla==0.9.15"` to
match the server version.  
**Files changed:** None (venv package only)  
**Status:** Resolved

---

## Issue #008 — carla.agents module not importable from venv

**Date:** 2026-11-11  
**Severity:** High (blocks BehaviorAgent import)  
**Description:** `from carla.agents.navigation.behavior_agent import BehaviorAgent`
failed with ModuleNotFoundError. The venv `carla` package (0.9.15 from PyPI)
does not include the `agents` subpackage, which lives in
`/home/alberto/carla/PythonAPI/carla/`.  
**Root cause:** The PyPI `carla` wheel is a client-only package; the full
PythonAPI (including agents) is only in the CARLA server installation.  
**Resolution:** Added `/home/alberto/carla/PythonAPI/carla/` to `sys.path`
(via `sys.path.append` in `run_closed_loop.py` and `cooperative_agent.py`)
so that `agents` is importable as a top-level package, while `carla` itself
is resolved from the venv site-packages.  
**Files changed:** `tools/carla/run_closed_loop.py`,
`tools/carla/cooperative_agent.py`  
**Status:** Resolved

---

## Issue #009 — BehaviorAgent.__init__() signature mismatch

**Date:** 2026-11-11  
**Severity:** Medium (blocks agent initialization)  
**Description:** `BehaviorAgent.__init__()` in CARLA 0.9.15 does not accept
`target_speed` as a keyword argument. Signature is
`__init__(self, vehicle, behavior='normal', opt_dict={}, map_inst=None, grp_inst=None)`.  
**Root cause:** Assumed `target_speed` was a constructor parameter (as in some
CARLA versions/documentation).  
**Resolution:** Call `super().__init__(vehicle, behavior='normal', **kwargs)`
without `target_speed`, then call `self.set_target_speed(target_speed)` after
initialization.  
**Files changed:** `tools/carla/cooperative_agent.py`  
**Status:** Resolved

---

## Issue #010 — BehaviorAgent private attribute names

**Date:** 2026-11-11  
**Severity:** Medium (blocks agent control loop)  
**Description:** `BehaviorAgent` stores the vehicle actor as `self._vehicle`
(not `self.vehicle`) and target speed as `self._target_speed` (not
`self.target_speed`). Additionally, `BehaviorAgent.run_step()` takes no
arguments.  
**Root cause:** Used public attribute names that do not exist in CARLA 0.9.15.  
**Resolution:** Replaced all `self.vehicle` → `self._vehicle` and
`self.target_speed` → `self._target_speed`. Changed
`super().run_step(target_speed=...)` to `super().run_step()` (no arguments),
setting `self._target_speed` directly before the call when speed reduction is
needed.  
**Files changed:** `tools/carla/cooperative_agent.py`  
**Status:** Resolved

---

## Issue #011 — Git push blocked by missing credentials

**Date:** 2026-11-11  
**Severity:** Medium (blocks remote backup)  
**Description:** `git push origin feature/phase-1-carla-integration` fails
because GitHub credentials (username + Personal Access Token) are not
configured in this session.  
**Root cause:** No GitHub authentication configured on this machine.  
**Resolution:** User must configure GitHub credentials (see instructions below)
and run `git push origin feature/phase-1-carla-integration`.  
**Files changed:** None  
**Status:** Pending user action

### How to configure GitHub credentials (for the user)

1. Generate a Personal Access Token on GitHub:
   Settings → Developer settings → Personal access tokens → Tokens (classic)
   → Generate new token with `repo` scope.

2. Configure git:
   ```bash
   cd /home/alberto/coperception-Integration
   git config --global credential.helper store
   git remote set-url origin https://TUO_USERNAME:IL_TUO_TOKEN@github.com/molifabio/coperception-Integration.git
   git push origin feature/phase-1-carla-integration
   ```

---

*End of ISSUES_AND_RESOLUTIONS.md*