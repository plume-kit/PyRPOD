"""Generate the inclined-plate target mesh for the Cai 2016 verification case.

Builds a single-sided, uniformly triangulated rectangular plate matching the
paper's Section-4 geometry (Aerospace 2016, 3(4):43): an 8 m x 8 m plate whose
center sits at (L, 0, 0) = (4, 0, 0) m from the nozzle exit, inclined by
alpha0 = 60 deg. The thruster fires along +X from a head-on JFH pose (VV at
the origin, identity DCM), so the global frame coincides with the paper's
nozzle frame and with pyrpod/plume/CaiImpingement2016.py:

    plate point(s, tau) = center + s * (0, 1, 0) + tau * (cos a0, 0, sin a0)

Every face normal points toward the thruster, (-sin a0, 0, cos a0), which the
strike pipeline's facing test (surface_dot_plume < 0) requires; the script
asserts this before saving.

Angle, distance, plate size, and mesh resolution are parametrized for the
Phase-3 sweep reuse. Defaults reproduce the paper case with 2 * 72^2 = 10368
faces (~0.11 m elements): contour-quality resolution at a per-face kinetics
cost that keeps a single firing in the tens of seconds.

Run from this directory:  python transform_inclined_plate.py
"""

import argparse
from pathlib import Path

import numpy as np
from stl import mesh


def build_plate_mesh(alpha0_deg=60.0, center=(4.0, 0.0, 0.0),
                     half_width=4.0, half_height=4.0, n_div=72):
    """Return a numpy-stl Mesh of the inclined plate.

    Parameters
    ----------
    alpha0_deg : float
        plate inclination angle alpha0 (deg); 90 = normal to the jet axis
    center : sequence of float
        plate center in the global/nozzle frame (m)
    half_width : float
        semi-width W0 along the horizontal s direction (m)
    half_height : float
        semi-length H0 along the inclined tau direction (m)
    n_div : int
        quad divisions per axis (faces = 2 * n_div^2)
    """
    a0 = np.deg2rad(alpha0_deg)
    center = np.asarray(center, dtype=float)
    t_s = np.array([0.0, 1.0, 0.0])
    t_tau = np.array([np.cos(a0), 0.0, np.sin(a0)])
    normal = np.array([-np.sin(a0), 0.0, np.cos(a0)])

    s_edges = np.linspace(-half_width, half_width, n_div + 1)
    tau_edges = np.linspace(-half_height, half_height, n_div + 1)

    def point(s, tau):
        return center + s * t_s + tau * t_tau

    data = np.zeros(2 * n_div * n_div, dtype=mesh.Mesh.dtype)
    k = 0
    for i in range(n_div):
        for j in range(n_div):
            p00 = point(s_edges[i], tau_edges[j])
            p10 = point(s_edges[i + 1], tau_edges[j])
            p01 = point(s_edges[i], tau_edges[j + 1])
            p11 = point(s_edges[i + 1], tau_edges[j + 1])
            # wound so cross(v1-v0, v2-v0) points along `normal`
            data['vectors'][k] = np.array([p00, p01, p11])
            data['vectors'][k + 1] = np.array([p00, p11, p10])
            k += 2

    plate = mesh.Mesh(data)
    plate.update_normals()
    unit_normals = plate.get_unit_normals()
    assert np.allclose(unit_normals, normal, atol=1e-9), (
        'face normals do not all point toward the thruster')
    return plate


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--alpha0-deg', type=float, default=60.0)
    parser.add_argument('--distance', type=float, default=4.0,
                        help='nozzle-to-plate-center distance L (m)')
    parser.add_argument('--half-width', type=float, default=4.0)
    parser.add_argument('--half-height', type=float, default=4.0)
    parser.add_argument('--n-div', type=int, default=72)
    parser.add_argument('--out', type=str,
                        default='inclined_plate_transformed.stl')
    args = parser.parse_args()

    plate = build_plate_mesh(alpha0_deg=args.alpha0_deg,
                             center=(args.distance, 0.0, 0.0),
                             half_width=args.half_width,
                             half_height=args.half_height,
                             n_div=args.n_div)
    out_path = Path(__file__).resolve().parent / args.out
    plate.save(str(out_path))
    print(f'saved {out_path} ({len(plate.vectors)} faces, '
          f'alpha0 = {args.alpha0_deg} deg, L = {args.distance} m)')
