# TimePico Bootstrap and OTA Update Manager
# Copyright 2026 - Matthew J. Riblett
#
# MicroPython (RP2040 / RP2350), intended for Raspberry Pi Pico W / Pico 2 W
#
# Minimal dependencies: MicroPython stdlib only.
#
# IMPORTANT:
# - The main application should be stored as: app.py
# - OTA uploads are written to: app_next.py and marked by update_pending.txt
# - If the candidate imports successfully, it is swapped into app.py on reboot.
# - Previous app is preserved as app_prev.py (one-level rollback).
#
# File on Pico:
#   main.py

import gc, machine, sys, time
import os

PENDING = "update_pending.txt"
APP = "app.py"
CAND = "app_next.py"
PREV = "app_prev.py"
BAD = "app_bad.py"
LOG = "boot_log.txt"

def _exists(p):
    try:
        os.stat(p)
        return True
    except Exception:
        return False

def _safe_remove(p):
    try:
        os.remove(p)
    except Exception:
        pass

def _safe_rename(src, dst):
    # Overwrite dst if it exists.
    try:
        _safe_remove(dst)
    except Exception:
        pass
    try:
        os.rename(src, dst)
        return True
    except Exception:
        return False

def _log(msg):
    try:
        with open(LOG, "a") as f:
            f.write("%s\n" % msg)
    except Exception:
        pass

def _try_import(modname):
    # Import module and ensure it defines main().
    gc.collect()
    try:
        mod = __import__(modname)
        if not hasattr(mod, "main"):
            raise RuntimeError("module has no main()")
        return True
    except Exception as e:
        _log("IMPORT FAIL %s: %r" % (modname, e))
        try:
            del sys.modules[modname]
        except Exception:
            pass
        gc.collect()
        return False

def _read_pending_candidate():
    # Returns filename (default app_next.py) if pending marker exists.
    if not _exists(PENDING):
        return None
    fn = CAND
    try:
        with open(PENDING, "r") as f:
            line = f.readline().strip()
            if line.endswith(".py"):
                fn = line
    except Exception:
        fn = CAND
    return fn

def _commit_update(candidate_py):
    # Candidate already validated by import.
    # Rotate previous backup only after validation.
    if _exists(PREV):
        _safe_remove(PREV)
    if _exists(APP):
        # Rename current app to backup (non-destructive)
        _safe_rename(APP, PREV)
    # Move candidate into place
    if not _safe_rename(candidate_py, APP):
        _log("COMMIT FAIL: rename candidate -> app.py")
        # Try to restore previous
        if _exists(PREV) and (not _exists(APP)):
            _safe_rename(PREV, APP)
        return False
    _safe_remove(PENDING)
    return True

# --- Apply staged update if present ---
cand = _read_pending_candidate()
if cand and _exists(cand):
    modname = cand[:-3] if cand.endswith(".py") else cand
    if _try_import(modname):
        _log("UPDATE OK: committing %s" % cand)
        if _commit_update(cand):
            _log("UPDATE COMMITTED. Rebooting...")
            time.sleep_ms(200)
            machine.reset()
    else:
        _log("UPDATE FAIL: removing candidate %s" % cand)
        _safe_remove(cand)
        _safe_remove(PENDING)
        time.sleep_ms(200)

# --- Run active application ---
if not _exists(APP):
    _log("FATAL: app.py missing")
    while True:
        time.sleep(1)

try:
    import app
    app.main()
except Exception as e:
    _log("APP CRASH: %r" % e)
    # If a previous version exists, restore it (import-time failures are the main target)
    if _exists(PREV):
        _safe_remove(BAD)
        _safe_rename(APP, BAD)
        _safe_rename(PREV, APP)
        _log("ROLLED BACK to app_prev.py. Rebooting...")
        time.sleep_ms(200)
        machine.reset()
    while True:
        time.sleep(1)
