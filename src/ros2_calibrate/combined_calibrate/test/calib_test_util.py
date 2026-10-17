"""Locate the combined_calibrate executables for the self checks.

The package can live in any workspace and in either the isolated or the merged
install layout, so look in PATH, COMBINED_CALIBRATE_PREFIX, AMENT_PREFIX_PATH
and finally ask `ros2 pkg prefix`.
"""
from __future__ import annotations

import os
import shutil
import subprocess

PACKAGE = "combined_calibrate"


def candidate_lib_dirs():
    for variable in ("COMBINED_CALIBRATE_PREFIX", "AMENT_PREFIX_PATH"):
        for prefix in os.environ.get(variable, "").split(os.pathsep):
            if not prefix:
                continue
            yield os.path.join(prefix, "lib", PACKAGE)            # <install>/...
            yield os.path.join(prefix, PACKAGE, "lib", PACKAGE)   # <ws>/install/...
    try:
        prefix = subprocess.run(["ros2", "pkg", "prefix", PACKAGE], capture_output=True,
                                text=True, check=True).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        prefix = ""
    if prefix:
        yield os.path.join(prefix, "lib", PACKAGE)


def find_tool(name):
    """Absolute path of a combined_calibrate executable, or None."""
    on_path = shutil.which(name)
    if on_path:
        return on_path
    for directory in candidate_lib_dirs():
        candidate = os.path.join(directory, name)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None
