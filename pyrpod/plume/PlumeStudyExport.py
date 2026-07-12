"""Small export/visualization helper for PlumeStrikeEstimationStudy.

This isolates file-writing and optional figure saving so the planner
can delegate those responsibilities and remain focused on simulation
logic.
"""
import os
from typing import Optional

import matplotlib.pyplot as plt


class PlumeStudyExport:
    """Helper service for saving meshes and figures used by the study.

    The actual mesh object is expected to implement a ``save(path)`` method
    (matching the existing usage in the codebase).
    """

    def __init__(self, environment):
        self.environment = environment

    def _ensure_parent(self, path: str) -> None:
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)

    def export_firing(self, vv_mesh, path_to_stl: str) -> None:
        """Save the provided mesh to disk, ensuring directory exists.

        Keeps behavior identical to previous calls to ``mesh.save(path)``
        but centralizes directory creation and any future enhancements.
        """
        self._ensure_parent(path_to_stl)
        vv_mesh.save(path_to_stl)

    def save_figure(self, figure: plt.Figure, image_path: str, close: bool = True) -> None:
        """Save a Matplotlib figure to disk and optionally close it.

        Ensures the output directory exists.
        """
        self._ensure_parent(image_path)
        figure.savefig(image_path)
        if close:
            plt.close(figure)
