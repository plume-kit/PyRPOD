"""
Plume impingement computations for RPOD.

Responsibilities:
- Given target mesh, VV pose, and active thrusters, compute per-face strike metrics
- Return numpy arrays/dicts; do not write files

This consolidates logic currently in RPOD.jfh_plume_strikes into
reusable, testable functions.

Implementation notes:
- compute_plume_strikes() runs a NumPy-vectorized strike-detection path by
  default. _compute_plume_strikes_scalar() preserves the original per-face
  loop verbatim as a reference implementation for tests and benchmarking.
- The vectorized core operates on plain serializable inputs (arrays, dicts,
  floats) so it can also run inside process-based workers.

Future work (no new dependencies planned):
- Vectorize the SimplifiedGasKinetics evaluations for struck faces.
- Shared-memory arrays (multiprocessing.shared_memory) for very large meshes.
- Chunking strategy to batch many small firings per worker task.
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
from pyrpod.plume.RarefiedPlumeGasKinetics import SimplifiedGasKinetics


def compute_face_centroids(vectors: np.ndarray) -> np.ndarray:
    """Compute per-face centroids for an (N x 3 x 3) array of face vertices.

    Averages the three vertices of each face, matching the scalar reference
    (mean over each coordinate in the face's native dtype). The target is
    stationary during a run, so callers should compute this once and pass it
    to compute_plume_strikes() via face_centroids.
    """
    return np.asarray(vectors).mean(axis=1)


def _build_thruster_link(thruster_data: Dict[str, Any]) -> Dict[str, Any]:
    """Map numeric JFH thruster indices ('1', '2', ...) to thruster names,
    consistent with legacy ordering of the thruster configuration."""
    link = {}
    i = 1
    for thruster in thruster_data:
        link[str(i)] = thruster_data[thruster]['name']
        i += 1
    return link


def extract_plume_params(environment: Any) -> Dict[str, Any]:
    """Extract the plain config values needed for strike computation.

    Returns a picklable dict (radius, wedge_theta, use_kinetics, and — only
    when kinetics is enabled — surface_temp and sigma) so workers never need
    the full environment object.
    """
    config = environment.config
    use_kinetics = config['pm']['kinetics'] != 'None'
    params: Dict[str, Any] = {
        'radius': float(config['plume']['radius']),
        'wedge_theta': float(config['plume']['wedge_theta']),
        'use_kinetics': use_kinetics,
        'surface_temp': None,
        'sigma': None,
    }
    if use_kinetics:
        params['surface_temp'] = float(config['tv']['surface_temp'])
        params['sigma'] = float(config['tv']['sigma'])
    return params


def _compute_plume_strikes_core(
    face_centroids: np.ndarray,
    target_unit_normals: np.ndarray,
    thruster_data: Dict[str, Any],
    thruster_metrics: Optional[Dict[str, Any]],
    jfh_step: Dict[str, Any],
    plume_params: Dict[str, Any],
) -> Dict[str, np.ndarray]:
    """Vectorized strike computation on plain serializable inputs.

    Geometry is evaluated with NumPy over all faces per active thruster.
    Gas-kinetics quantities remain scalar: SimplifiedGasKinetics is
    instantiated only for struck face indices, exactly as in the scalar
    reference. Memory scales with the number of faces (a few (N,) and (N,3)
    temporaries), independent of the number of firings.
    """
    num_faces = len(face_centroids)
    strikes = np.zeros(num_faces)

    use_kinetics = plume_params['use_kinetics']
    if use_kinetics:
        pressures = np.zeros(num_faces)
        shear_stresses = np.zeros(num_faces)
        heat_flux = np.zeros(num_faces)
        heat_flux_load = np.zeros(num_faces)

    vv_pos = np.array(jfh_step['xyz'])
    vv_orientation = np.array(jfh_step['dcm']).transpose()
    thrusters = jfh_step['thrusters']
    firing_time = float(jfh_step['t']) if 't' in jfh_step else 0.0

    link = _build_thruster_link(thruster_data)

    plume_radius = float(plume_params['radius'])
    wedge_theta = float(plume_params['wedge_theta'])

    normals = np.asarray(target_unit_normals)

    for thr in thrusters:
        thruster_id = link[str(thr)][0]

        thruster_orientation = np.array(thruster_data[thruster_id]['dcm']).transpose()
        thruster_orientation = thruster_orientation.dot(vv_orientation)
        plume_normal = np.array(thruster_orientation[0])
        norm_plume_normal = np.linalg.norm(plume_normal)
        unit_plume_normal = plume_normal / norm_plume_normal

        thr_exit = np.array(thruster_data[thruster_id]['exit'])
        thruster_pos = vv_pos + thr_exit
        thruster_pos = thruster_pos[0]

        distance = thruster_pos - face_centroids
        norm_distance = np.linalg.norm(distance, axis=1)

        # Faces whose centroid coincides with the thruster exit are skipped,
        # matching the scalar reference's `norm_distance == 0` guard.
        valid = norm_distance != 0.0
        unit_distance = np.zeros_like(distance)
        np.divide(
            distance,
            norm_distance[:, np.newaxis],
            out=unit_distance,
            where=valid[:, np.newaxis],
        )

        # NOTE: 3.14 (not np.pi) is kept deliberately to reproduce the legacy
        # scalar reference bit-for-bit; changing it shifts theta by ~1.6e-3 rad
        # and can alter struck-face IDs near the wedge boundary.
        theta = 3.14 - np.arccos((unit_distance * unit_plume_normal).sum(axis=1))

        surface_dot_plume = (normals * unit_plume_normal).sum(axis=1)

        hit = (
            valid
            & (norm_distance < plume_radius)
            & (theta < wedge_theta)
            & (surface_dot_plume < 0)
        )

        strikes[hit] += 1

        if use_kinetics:
            T_w = plume_params['surface_temp']
            sigma = plume_params['sigma']
            t_type = thruster_data[thruster_id]['type'][0]
            metrics = thruster_metrics[t_type]
            for idx in np.nonzero(hit)[0]:
                simple_plume = SimplifiedGasKinetics(
                    norm_distance[idx], theta[idx], metrics, T_w, sigma
                )
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


def compute_plume_strikes(
    target_mesh: Any,
    target_unit_normals: np.ndarray,
    vv: Any,
    jfh_step: Dict[str, Any],
    environment: Any,
    face_centroids: Optional[np.ndarray] = None,
) -> Dict[str, np.ndarray]:
    """Compute plume strike arrays for a single JFH step.

    Inputs
    - target_mesh: numpy-stl Mesh-like, exposes .vectors (N x 3 x 3)
    - target_unit_normals: (N x 3) array of per-face unit normals
    - vv: Visiting vehicle with thruster_data and thruster_metrics
    - jfh_step: dict with keys 'thrusters' (list[int]), 'xyz' (pos), 'dcm' (3x3)
    - environment: provides config for plume and kinetics
    - face_centroids: optional (N x 3) precomputed face centroids
      (see compute_face_centroids). When the target is stationary, callers
      should compute centroids once per run and pass them here; if omitted,
      they are computed from target_mesh for this step.

    Returns
    - dict with per-face arrays for current step: strikes and optionally pressures, shear_stress, heat_flux_rate, heat_flux_load
    """
    if face_centroids is None:
        face_centroids = compute_face_centroids(target_mesh.vectors)
    plume_params = extract_plume_params(environment)
    return _compute_plume_strikes_core(
        face_centroids=face_centroids,
        target_unit_normals=target_unit_normals,
        thruster_data=vv.thruster_data,
        # Only defined/needed when kinetics is enabled; the core only reads it
        # for struck faces, matching the scalar reference.
        thruster_metrics=getattr(vv, 'thruster_metrics', None),
        jfh_step=jfh_step,
        plume_params=plume_params,
    )


def _compute_plume_strikes_scalar(
    target_mesh: Any,
    target_unit_normals: np.ndarray,
    vv: Any,
    jfh_step: Dict[str, Any],
    environment: Any,
) -> Dict[str, np.ndarray]:
    """Scalar reference implementation of compute_plume_strikes().

    Preserved verbatim from the original per-face loop. Kept for regression
    tests, debugging, and benchmarking against the vectorized path; the two
    must produce identical strike arrays and struck-face IDs.
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


# Per-process state for parallel workers. Populated once per worker by
# _parallel_worker_init so the (N,3) target arrays are shipped to each worker
# a single time instead of once per submitted firing. Memory therefore scales
# with workers x faces, never firings x faces.
_WORKER_STATE: Dict[str, Any] = {}


def _parallel_worker_init(
    face_centroids: np.ndarray,
    target_unit_normals: np.ndarray,
    thruster_data: Dict[str, Any],
    thruster_metrics: Optional[Dict[str, Any]],
    plume_params: Dict[str, Any],
) -> None:
    """ProcessPoolExecutor initializer: cache shared per-run inputs."""
    _WORKER_STATE['face_centroids'] = face_centroids
    _WORKER_STATE['target_unit_normals'] = target_unit_normals
    _WORKER_STATE['thruster_data'] = thruster_data
    _WORKER_STATE['thruster_metrics'] = thruster_metrics
    _WORKER_STATE['plume_params'] = plume_params


def _parallel_worker_compute(task) -> Any:
    """Compute strikes for one firing inside a worker process.

    task is (firing_index, jfh_step); returns (firing_index, result dict).
    """
    firing_index, jfh_step = task
    result = _compute_plume_strikes_core(
        face_centroids=_WORKER_STATE['face_centroids'],
        target_unit_normals=_WORKER_STATE['target_unit_normals'],
        thruster_data=_WORKER_STATE['thruster_data'],
        thruster_metrics=_WORKER_STATE['thruster_metrics'],
        jfh_step=jfh_step,
        plume_params=_WORKER_STATE['plume_params'],
    )
    return firing_index, result


def run_parallel_plume_strikes(
    jfh_steps: Sequence[Dict[str, Any]],
    face_centroids: np.ndarray,
    target_unit_normals: np.ndarray,
    thruster_data: Dict[str, Any],
    thruster_metrics: Optional[Dict[str, Any]],
    plume_params: Dict[str, Any],
    workers: int,
) -> List[Dict[str, np.ndarray]]:
    """Compute per-firing strike results across processes, one firing per task.

    All inputs must be plain serializable data (NumPy arrays, dicts,
    primitives) — full study/vehicle/environment objects are never pickled.
    Results are returned as a list indexed by firing, preserving JFH order
    regardless of completion order; cumulative accumulation and VTK output
    remain the caller's responsibility (serial, in the parent process).

    Raises whatever the executor or workers raise; callers are expected to
    fall back to the serial path with a clear message.
    """
    results: List[Optional[Dict[str, np.ndarray]]] = [None] * len(jfh_steps)
    with ProcessPoolExecutor(
        max_workers=workers,
        initializer=_parallel_worker_init,
        initargs=(
            face_centroids,
            target_unit_normals,
            thruster_data,
            thruster_metrics,
            plume_params,
        ),
    ) as executor:
        futures = [
            executor.submit(_parallel_worker_compute, (i, step))
            for i, step in enumerate(jfh_steps)
        ]
        for future in futures:
            firing_index, result = future.result()
            results[firing_index] = result
    return results


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
