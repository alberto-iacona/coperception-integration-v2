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

*End of ISSUES_AND_RESOLUTIONS.md*