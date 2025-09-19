"""
IO helpers for RPOD: VTK/STL writers and results directory setup.

Responsibilities:
- Create case results subdirectories
- Save meshes to STL/VTK
- Write JFH files (wraps util.io.file_print)

Keep logic thin and parameterized; do not compute physics here.
"""
from __future__ import annotations

import os
from typing import Any, Dict, Sequence

from pyrpod.util.io.file_print import print_JFH, print_1d_JFH


def ensure_results_dirs(case_dir: str, subdirs: Sequence[str] = ("results", "results/strikes", "results/jfh")) -> None:
    for sub in subdirs:
        os.makedirs(os.path.join(case_dir, sub), exist_ok=True)


def save_mesh_to_stl(mesh_obj: Any, path: str) -> None:
    """Save a numpy-stl Mesh-like object to STL."""
    # mesh_obj is expected to have .save(path)
    mesh_obj.save(path)


def write_jfh(t_values, r, rot, path: str, mode: str = "1d") -> None:
    """Write a JFH file.

    mode="1d" uses print_1d_JFH formatting; mode="generic" uses print_JFH.
    """
    if mode == "1d":
        print_1d_JFH(t_values, r, rot, path)
    else:
        print_JFH(t_values, r, rot, path)
