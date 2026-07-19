# ========================
# PyRPOD: tests/rpod/rpod_integration_test_06.py
# ========================
# Performance benchmark for plume strike computation on existing RPOD cases:
#   - scalar reference vs NumPy-vectorized geometry
#     (1d_approach: geometry only; multi_thrusters_square: Simplified kinetics)
#   - serial vs process-parallel execution across JFH firings
#
# Elapsed wall-clock times are printed (run pytest with -s to see them); no
# speedup thresholds are asserted, so timing noise on shared/CI machines can
# never fail the build. Each benchmark only asserts that the compared paths
# produce identical strike arrays, i.e. that the benchmark code ran
# successfully. Correctness itself is covered by
# tests/plume/plume_unit_test_04.py and tests/plume/plume_integration_test_02.py.
#
# Note: the parallel benchmark includes worker process startup (on Windows,
# spawn re-imports pyrpod/scipy per worker), which dominates on the small
# repository cases; parallel execution is intended for large meshes and/or
# many firings.

import time
import unittest

import numpy as np

from pyrpod.mission import MissionEnvironment
from pyrpod.plume.PlumeStrikeCalculator import (
    _compute_plume_strikes_scalar,
    compute_face_centroids,
    compute_plume_strikes,
    extract_plume_params,
    run_parallel_plume_strikes,
)
from pyrpod.rpod import JetFiringHistory
from pyrpod.vehicle import TargetVehicle, VisitingVehicle


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


def build_steps(jfh):
    return [
        {
            'thrusters': jfh.JFH[firing]['thrusters'],
            'xyz': np.array(jfh.JFH[firing]['xyz']),
            'dcm': np.array(jfh.JFH[firing]['dcm']),
            't': float(jfh.JFH[firing]['t']),
        }
        for firing in range(len(jfh.JFH))
    ]


def timed(fn):
    start = time.perf_counter()
    result = fn()
    return time.perf_counter() - start, result


class PlumeStrikeBenchmarkChecks(unittest.TestCase):

    def benchmark_scalar_vs_vectorized(self, case_dir):
        jfh, tv, vv, me = load_case(case_dir)
        target = tv.mesh
        normals = target.get_unit_normals()
        centroids = compute_face_centroids(target.vectors)
        steps = build_steps(jfh)

        t_vec, vec_results = timed(lambda: [
            compute_plume_strikes(target, normals, vv, step, me,
                                  face_centroids=centroids)
            for step in steps
        ])
        t_scalar, scalar_results = timed(lambda: [
            _compute_plume_strikes_scalar(target, normals, vv, step, me)
            for step in steps
        ])

        print(f"\n[benchmark] {case_dir} "
              f"({len(steps)} firings x {len(target.vectors)} faces, "
              f"kinetics: {me.config['pm']['kinetics']})")
        print(f"[benchmark]   vectorized (serial): {t_vec:8.3f} s")
        print(f"[benchmark]   scalar reference:    {t_scalar:8.3f} s "
              f"({t_scalar / t_vec:5.1f}x slower than vectorized)")

        # Success criterion: both paths ran and agree exactly.
        for vec, ref in zip(vec_results, scalar_results):
            np.testing.assert_array_equal(vec['strikes'], ref['strikes'])

    def test_scalar_vs_vectorized_geometry(self):
        self.benchmark_scalar_vs_vectorized('../case/rpod/1d_approach/')

    def test_scalar_vs_vectorized_kinetics(self):
        self.benchmark_scalar_vs_vectorized('../case/rpod/multi_thrusters_square/')

    def test_serial_vs_parallel(self):
        case_dir = '../case/rpod/1d_approach/'
        jfh, tv, vv, me = load_case(case_dir)
        target = tv.mesh
        normals = target.get_unit_normals()
        centroids = compute_face_centroids(target.vectors)
        steps = build_steps(jfh)
        workers = min(2, len(steps))

        t_serial, serial_results = timed(lambda: [
            compute_plume_strikes(target, normals, vv, step, me,
                                  face_centroids=centroids)
            for step in steps
        ])
        t_par, par_results = timed(lambda: run_parallel_plume_strikes(
            jfh_steps=steps,
            face_centroids=centroids,
            target_unit_normals=normals,
            thruster_data=vv.thruster_data,
            thruster_metrics=getattr(vv, 'thruster_metrics', None),
            plume_params=extract_plume_params(me),
            workers=workers,
        ))

        print(f"\n[benchmark] {case_dir} ({len(steps)} firings)")
        print(f"[benchmark]   serial (vectorized):   {t_serial:8.3f} s")
        print(f"[benchmark]   parallel ({workers} workers):  {t_par:8.3f} s "
              f"(includes process startup)")

        # Success criterion: both paths ran and agree exactly.
        for ser, par in zip(serial_results, par_results):
            np.testing.assert_array_equal(ser['strikes'], par['strikes'])


if __name__ == '__main__':
    unittest.main()
