"""
Approach maneuvers: 1D axial burn profiles, mass/prop evolution, and JFH generation helpers.

Design goals:
- Compute-first API: return arrays; no file I/O
- Reuse FuelManager (delta-v, delta-mass) and ThrusterGrouping (m_dot, v_e)
- Allow pluggable time-step strategies (fixed, heuristic, min-impulse-bit-based)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Tuple
import numpy as np


@dataclass
class ApproachInputs:
    v_ida: float  # docking velocity target
    v_o: float    # initial axial velocity (> v_ida)
    r_o: float    # initial range along docking axis (m)
    group: str = "neg_x"  # thruster group for decel


def choose_dt_from_mib(vv: Any, group: str, time_multiplier: float = 1.0) -> float:
    """Default time step based on min impulse bit and a multiplier.

    dt ~ MIB / F_thruster scaled by time_multiplier, using the first thruster in the group.
    """
    thr_name = vv.rcs_groups[group][0]
    t_type = vv.thruster_data[thr_name]['type'][0]
    MIB = vv.thruster_metrics[t_type]['MIB']
    F_thruster = vv.thruster_metrics[t_type]['F']
    return (MIB / F_thruster) * time_multiplier


def heuristic_time_multiplier(vv: Any, fuel_mgr: Any, group: str, v_ida: float, v_o: float, r_o: float) -> float:
    """Run a coarse sim to estimate a time multiplier. Mirrors RPOD.calc_time_multiplier behavior."""
    # This will be implemented by porting logic from RPOD.calc_time_multiplier during refactor
    raise NotImplementedError("heuristic_time_multiplier pending port from RPOD.calc_time_multiplier")


def compute_1d_approach(
    inputs: ApproachInputs,
    vv: Any,
    fuel_mgr: Any,
    grouping: Any,
    cant_rad: float,
    dt_strategy: Dict[str, Any] | None = None,
) -> Dict[str, np.ndarray]:
    """Discrete 1D deceleration under constant-thrust firings.

    Returns dict with arrays: t, x, y, z, v, dv, mass, dm_total, rot
    No file I/O; callers can serialize via rpod.io.write_jfh.
    """
    v_ida, v_o, r_o = inputs.v_ida, inputs.v_o, inputs.r_o
    group = inputs.group

    # Pre-compute thrust/mass-flow characteristics
    m_dot_sum = grouping.calc_m_dot_sum(group)
    v_e = grouping.calc_v_e(group)

    # Effective thrust with cant and number of thrusters
    n_thrusters = len(vv.rcs_groups[group])
    t_name0 = vv.rcs_groups[group][0]
    t_type0 = vv.thruster_data[t_name0]['type'][0]
    F_thruster = vv.thruster_metrics[t_type0]['F']
    F_eff = np.cos(cant_rad) * F_thruster * n_thrusters

    # Choose dt
    if dt_strategy and dt_strategy.get("type") == "heuristic":
        tm = dt_strategy.get("multiplier")
        if tm is None:
            raise NotImplementedError("heuristic time multiplier not yet wired; pass explicit multiplier")
        dt = choose_dt_from_mib(vv, group, time_multiplier=tm)
    else:
        tm = (dt_strategy or {}).get("multiplier", 1.0)
        dt = choose_dt_from_mib(vv, group, time_multiplier=tm)

    dm_firing = m_dot_sum * dt

    dv_req = v_o - v_ida

    # Initialize arrays
    x = [r_o]
    y = [0.0]
    z = [0.0]
    dx = [0.0]
    t = [0.0]
    dv = [0.0]
    v = [v_o]
    mass = [vv.mass]
    dm_total = [dm_firing]
    rot = []

    # Identity rotation placeholder (align x-axis)
    rot_mat = np.eye(3)
    rot.append(rot_mat)

    while dv_req > 0:
        m_o = mass[-1]
        mass.append(m_o - dm_firing)

        dv_f = fuel_mgr.calc_delta_v(dt, v_e, m_dot_sum, m_o)
        dv.append(dv_f)
        v.append(v[-1] - dv_f)

        v_avg = 0.5 * (v[-1] + v[-2])
        dx.append(v_avg * dt)
        x.append(x[-1] - v_avg * dt)
        y.append(0.0)
        z.append(0.0)

        dv_req -= dv_f
        dm_total.append(dm_total[-1] + dm_firing)
        t.append(t[-1] + dt)
        rot.append(rot_mat)

        if len(t) > 100000:
            # guard against runaway loops
            break

    return {
        "t": np.array(t),
        "x": np.array(x),
        "y": np.array(y),
        "z": np.array(z),
        "dx": np.array(dx),
        "v": np.array(v),
        "dv": np.array(dv),
        "mass": np.array(mass),
        "dm_total": np.array(dm_total),
        "rot": np.array(rot),
    }
