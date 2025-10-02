"""Mesh and visualization utilities for plume study plotting.

Centralizes common operations used by PlumeStrikeEstimationStudy so the
main class can stay focused on simulation logic.
"""
from typing import Dict, Optional, Sequence
import os
import numpy as np
from stl import mesh


def transform_plume_for_thruster(plume_mesh: mesh.Mesh, thruster_id: str, vv, vv_orientation: np.ndarray, vv_pos) -> mesh.Mesh:
    """Apply the standard sequence of rotations and translations to a plume mesh.

    Sequence mirrors legacy code:
    1. rotate by thruster DCM (transpose)
    2. rotate by VV orientation (transpose)
    3. translate by VV position
    4. if using clusters: translate by cluster exit for the thruster's cluster
    5. translate by thruster exit

    Returns the transformed mesh (mutates the provided mesh as the stl.mesh API does).
    """
    # First: thruster DCM in thruster coord frame
    thruster_orientation = np.array(vv.thruster_data[thruster_id]['dcm'])
    plume_mesh.rotate_using_matrix(thruster_orientation.transpose())

    # Second: according to DCM of VV in JFH
    plume_mesh.rotate_using_matrix(vv_orientation.transpose())

    # Third: according to position vector of the VV in JFH
    plume_mesh.translate(vv_pos)

    # Fourth: cluster offset if present
    if getattr(vv, 'use_clusters', False):
        cluster_key = thruster_id[0] + thruster_id[1] if len(thruster_id) >= 2 else thruster_id
        if cluster_key in vv.cluster_data:
            plume_mesh.translate(vv.cluster_data[cluster_key]['exit'][0])

    # Fifth: thruster exit vector
    plume_mesh.translate(vv.thruster_data[thruster_id]['exit'][0])

    return plume_mesh


def compose_meshes(meshes: Sequence[Optional[mesh.Mesh]]) -> Optional[mesh.Mesh]:
    """Concatenate non-None meshes into a single mesh.Mesh or return None.

    Avoids repeated concatenation logic in callers.
    """
    valid = [m.data for m in meshes if m is not None]
    if not valid:
        return None
    if len(valid) == 1:
        return meshes[0]
    # concatenate arrays of shape (n,3,3)
    combined = mesh.Mesh(np.concatenate(valid))
    return combined
