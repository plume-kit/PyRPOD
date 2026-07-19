# ========================
# PyRPOD: tests/plume/plume_unit_test_04.py
# ========================
# Asserts that the NumPy-vectorized plume strike detection in
# compute_plume_strikes() reproduces the scalar reference implementation
# (_compute_plume_strikes_scalar) exactly, for every firing of two
# representative existing cases:
#   - case/rpod/1d_approach          (kinetics disabled, geometry only)
#   - case/rpod/multi_thrusters_square (Simplified kinetics, multiple thrusters)
#
# Strike arrays and struck-face IDs must match exactly (integer counts from
# strict geometric comparisons). Kinetics arrays are compared with
# np.allclose(rtol=1e-12, atol=1e-12): on the tested platform they are
# bit-for-bit identical (both paths feed identical scalar inputs to
# SimplifiedGasKinetics), but the tolerance guards against BLAS/SIMD
# reduction-order differences on other platforms without weakening the test
# in any practically meaningful way.
#
# These functions compute arrays only; no files are written.

import unittest

import numpy as np

from pyrpod.mission import MissionEnvironment
from pyrpod.plume.PlumeStrikeCalculator import (
    _compute_plume_strikes_scalar,
    compute_face_centroids,
    compute_plume_strikes,
)
from pyrpod.rpod import JetFiringHistory
from pyrpod.vehicle import TargetVehicle, VisitingVehicle

KINETICS_RTOL = 1e-12
KINETICS_ATOL = 1e-12
KINETICS_KEYS = ("pressures", "shear_stress", "heat_flux_rate", "heat_flux_load")


def load_case(case_dir):
    jfh = JetFiringHistory.JetFiringHistory(case_dir)
    jfh.read_jfh()

    tv = TargetVehicle.TargetVehicle(case_dir)
    tv.set_stl()

    vv = VisitingVehicle.VisitingVehicle(case_dir)
    vv.set_thruster_config()
    vv.set_thruster_metrics()

    me = MissionEnvironment.MissionEnvironment(case_dir)
    return jfh, tv, vv, me


def build_step(jfh, firing):
    return {
        'thrusters': jfh.JFH[firing]['thrusters'],
        'xyz': np.array(jfh.JFH[firing]['xyz']),
        'dcm': np.array(jfh.JFH[firing]['dcm']),
        't': float(jfh.JFH[firing]['t']),
    }


class VectorizedMatchesScalarChecks(unittest.TestCase):

    def assert_case_equivalence(self, case_dir):
        jfh, tv, vv, me = load_case(case_dir)
        target = tv.mesh
        normals = target.get_unit_normals()
        centroids = compute_face_centroids(target.vectors)

        for firing in range(len(jfh.JFH)):
            step = build_step(jfh, firing)

            scalar = _compute_plume_strikes_scalar(target, normals, vv, step, me)
            vectorized = compute_plume_strikes(
                target, normals, vv, step, me, face_centroids=centroids
            )
            # Backward-compatible call without precomputed centroids.
            vectorized_no_centroids = compute_plume_strikes(
                target, normals, vv, step, me
            )

            for vec in (vectorized, vectorized_no_centroids):
                self.assertEqual(set(vec.keys()), set(scalar.keys()))

                # Exact strike counts and exact struck-face IDs are required.
                np.testing.assert_array_equal(
                    vec["strikes"], scalar["strikes"],
                    err_msg=f"{case_dir} firing {firing}: strikes differ",
                )
                np.testing.assert_array_equal(
                    np.nonzero(vec["strikes"])[0],
                    np.nonzero(scalar["strikes"])[0],
                    err_msg=f"{case_dir} firing {firing}: struck-face IDs differ",
                )

                # Kinetics arrays: tolerance documented in the module header.
                for key in KINETICS_KEYS:
                    if key in scalar:
                        self.assertTrue(
                            np.allclose(
                                vec[key], scalar[key],
                                rtol=KINETICS_RTOL, atol=KINETICS_ATOL,
                            ),
                            msg=f"{case_dir} firing {firing}: {key} differs",
                        )

    def test_geometry_only_case(self):
        self.assert_case_equivalence('../case/rpod/1d_approach/')

    def test_kinetics_multi_thruster_case(self):
        self.assert_case_equivalence('../case/rpod/multi_thrusters_square/')

    def test_zero_distance_face_skipped(self):
        # A face whose centroid coincides with the thruster exit must be
        # skipped by both implementations (legacy `norm_distance == 0` guard).
        case_dir = '../case/rpod/1d_approach/'
        jfh, tv, vv, me = load_case(case_dir)
        target = tv.mesh
        normals = target.get_unit_normals()

        step = build_step(jfh, 0)
        # Place the vehicle so the first thruster exit lands exactly on the
        # centroid of face 0.
        centroid0 = compute_face_centroids(target.vectors)[0].astype(np.float64)
        thruster_id = vv.thruster_data[next(iter(vv.thruster_data))]['name'][0]
        exit_pos = np.array(vv.thruster_data[thruster_id]['exit'])[0]
        step['xyz'] = centroid0 - exit_pos

        scalar = _compute_plume_strikes_scalar(target, normals, vv, step, me)
        vectorized = compute_plume_strikes(target, normals, vv, step, me)

        self.assertEqual(scalar["strikes"][0], 0.0)
        np.testing.assert_array_equal(vectorized["strikes"], scalar["strikes"])


if __name__ == '__main__':
    unittest.main()
