"""
Plume impingement computations for RPOD.

Responsibilities:
- Given target mesh, VV pose, and active thrusters, compute per-face strike metrics
- Return numpy arrays/dicts; do not write files

This consolidates logic currently in RPOD.jfh_plume_strikes into
reusable, testable functions.
"""
from __future__ import annotations

from typing import Any, Dict, Tuple
import numpy as np
from pyrpod.plume.RarefiedPlumeGasKinetics import SimplifiedGasKinetics


def compute_plume_strikes(
    target_mesh: Any,
    target_unit_normals: np.ndarray,
    vv: Any,
    jfh_step: Dict[str, Any],
    environment: Any,
) -> Dict[str, np.ndarray]:
    """Compute plume strike arrays for a single JFH step.

    Inputs
    - target_mesh: numpy-stl Mesh-like, exposes .vectors (N x 3 x 3)
    - target_unit_normals: (N x 3) array of per-face unit normals
    - vv: Visiting vehicle with thruster_data and thruster_metrics
    - jfh_step: dict with keys 'thrusters' (list[int]), 'xyz' (pos), 'dcm' (3x3)
    - environment: provides config for plume and kinetics

    Returns
    - dict with per-face arrays for current step: strikes and optionally pressures, shear_stress, heat_flux_rate, heat_flux_load
    """
    num_faces = len(target_mesh.vectors)
    strikes = np.zeros(num_faces)

    use_kinetics = environment.config['pm']['kinetics'] != 'None'
    if use_kinetics:
        pressures = np.zeros(num_faces)
        shear_stresses = np.zeros(num_faces)
        heat_flux = np.zeros(num_faces)
        heat_flux_load = np.zeros(num_faces)

    vv_pos = np.array(jfh_step['xyz'])
    vv_orientation = np.array(jfh_step['dcm']).transpose()
    thrusters = jfh_step['thrusters']
    firing_time = float(jfh_step['t']) if 't' in jfh_step else 0.0

    # Build mapping from numeric JFH indices to thruster ids consistent with legacy
    link = {}
    i = 1
    for thruster in vv.thruster_data:
        link[str(i)] = vv.thruster_data[thruster]['name']
        i += 1

    plume_radius = float(environment.config['plume']['radius'])
    wedge_theta = float(environment.config['plume']['wedge_theta'])

    for thr in thrusters:
        thruster_id = link[str(thr)][0]

        thruster_orientation = np.array(vv.thruster_data[thruster_id]['dcm']).transpose()
        thruster_orientation = thruster_orientation.dot(vv_orientation)
        plume_normal = np.array(thruster_orientation[0])
        norm_plume_normal = np.linalg.norm(plume_normal)
        unit_plume_normal = plume_normal / norm_plume_normal

        thr_exit = np.array(vv.thruster_data[thruster_id]['exit'])
        thruster_pos = vv_pos + thr_exit
        thruster_pos = thruster_pos[0]

        for idx, face in enumerate(target_mesh.vectors):
            face = np.array(face).transpose()
            centroid = np.array([face[0].mean(), face[1].mean(), face[2].mean()])
            distance = thruster_pos - centroid
            norm_distance = np.linalg.norm(distance)
            if norm_distance == 0:
                continue
            unit_distance = distance / norm_distance

            theta = 3.14 - np.arccos(np.dot(np.squeeze(unit_distance), np.squeeze(unit_plume_normal)))

            n = np.squeeze(target_unit_normals[idx])
            unit_plume = np.squeeze(plume_normal / norm_plume_normal)
            surface_dot_plume = np.dot(n, unit_plume)

            within_distance = float(norm_distance) < plume_radius
            within_theta = float(theta) < wedge_theta
            facing_thruster = surface_dot_plume < 0

            if within_distance and within_theta and facing_thruster:
                strikes[idx] += 1
                if use_kinetics:
                    T_w = float(environment.config['tv']['surface_temp'])
                    sigma = float(environment.config['tv']['sigma'])
                    t_type = vv.thruster_data[thruster_id]['type'][0]
                    thruster_metrics = vv.thruster_metrics[t_type]
                    simple_plume = SimplifiedGasKinetics(norm_distance, theta, thruster_metrics, T_w, sigma)
                    pressures[idx] += simple_plume.get_pressure()
                    shear = simple_plume.get_shear_pressure()
                    shear_stresses[idx] += abs(shear)
                    hf = simple_plume.get_heat_flux()
                    heat_flux[idx] += hf
                    heat_flux_load[idx] += hf * firing_time

    result = {"strikes": strikes}
    if use_kinetics:
        result.update({
            "pressures": pressures,
            "shear_stress": shear_stresses,
            "heat_flux_rate": heat_flux,
            "heat_flux_load": heat_flux_load,
        })
    return result


def accumulate_cumulative(
    cumulative: Dict[str, np.ndarray],
    current: Dict[str, np.ndarray],
) -> Dict[str, np.ndarray]:
    """Accumulate per-step arrays into cumulative tallies (e.g., cum_strikes, max_pressures)."""
    if "cum_strikes" in cumulative and "strikes" in current:
        cumulative["cum_strikes"] = cumulative["cum_strikes"] + current["strikes"]

    # Max trackers if available
    if "max_pressures" in cumulative and "pressures" in current:
        cumulative["max_pressures"] = np.maximum(cumulative["max_pressures"], current["pressures"])
    if "max_shears" in cumulative and "shear_stress" in current:
        cumulative["max_shears"] = np.maximum(cumulative["max_shears"], current["shear_stress"])

    if "cum_heat_flux_load" in cumulative and "heat_flux_load" in current:
        cumulative["cum_heat_flux_load"] = cumulative["cum_heat_flux_load"] + current["heat_flux_load"]

    return cumulative
