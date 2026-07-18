"""
Benchmark plume strike computation: scalar reference vs vectorized geometry,
and serial vs optional process-parallel execution across JFH firings.

Usage (from the repository root):
    python scripts/benchmark_plume_strikes.py
    python scripts/benchmark_plume_strikes.py --case case/rpod/multi_thrusters_square/
    python scripts/benchmark_plume_strikes.py --workers 4 --repeat 3

Reports elapsed wall-clock times only; it asserts result equivalence but no
speedup thresholds, so it stays robust on shared/CI machines. Correctness is
covered by tests/plume/plume_unit_test_04.py and plume_integration_test_02.py.

Uses only cases and data already present in the repository; no new
dependencies (NumPy + standard library).
"""
import argparse
import os
import sys
import time

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pyrpod.mission import MissionEnvironment                     # noqa: E402
from pyrpod.plume.PlumeStrikeCalculator import (                  # noqa: E402
    _compute_plume_strikes_scalar,
    compute_face_centroids,
    compute_plume_strikes,
    extract_plume_params,
    run_parallel_plume_strikes,
)
from pyrpod.rpod import JetFiringHistory                          # noqa: E402
from pyrpod.vehicle import TargetVehicle, VisitingVehicle         # noqa: E402


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


def time_best_of(fn, repeat):
    best = float('inf')
    result = None
    for _ in range(repeat):
        start = time.perf_counter()
        result = fn()
        best = min(best, time.perf_counter() - start)
    return best, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--case', default='case/rpod/1d_approach/',
        help='Case directory (relative to the repo root or absolute).',
    )
    parser.add_argument(
        '--workers', type=int, default=min(os.cpu_count() or 1, 4),
        help='Worker processes for the parallel benchmark (default: up to 4).',
    )
    parser.add_argument(
        '--repeat', type=int, default=1,
        help='Repetitions per measurement; best time is reported.',
    )
    parser.add_argument(
        '--skip-scalar', action='store_true',
        help='Skip the (slow) scalar reference benchmark.',
    )
    args = parser.parse_args()

    case_dir = args.case
    if not os.path.isabs(case_dir):
        case_dir = os.path.join(REPO_ROOT, case_dir)
    case_dir = case_dir.replace('\\', '/')
    if not case_dir.endswith('/'):
        case_dir += '/'

    jfh, tv, vv, me = load_case(case_dir)
    target = tv.mesh
    normals = target.get_unit_normals()
    centroids = compute_face_centroids(target.vectors)
    steps = build_steps(jfh)
    n_firings = len(steps)
    n_faces = len(target.vectors)

    print(f"case:    {case_dir}")
    print(f"firings: {n_firings}, faces: {n_faces}, "
          f"kinetics: {me.config['pm']['kinetics']}")
    print(f"repeat:  {args.repeat} (best time reported)\n")

    # --- Scalar reference vs vectorized geometry (per-firing compute only) ---
    def run_vectorized():
        return [
            compute_plume_strikes(target, normals, vv, step, me,
                                  face_centroids=centroids)
            for step in steps
        ]

    t_vec, vec_results = time_best_of(run_vectorized, args.repeat)
    print(f"vectorized (serial):   {t_vec:8.3f} s")

    if not args.skip_scalar:
        def run_scalar():
            return [
                _compute_plume_strikes_scalar(target, normals, vv, step, me)
                for step in steps
            ]

        t_scalar, scalar_results = time_best_of(run_scalar, args.repeat)
        print(f"scalar reference:      {t_scalar:8.3f} s"
              f"   ({t_scalar / t_vec:5.1f}x slower than vectorized)")
        for vec, ref in zip(vec_results, scalar_results):
            assert np.array_equal(vec['strikes'], ref['strikes']), \
                "vectorized/scalar strike mismatch"

    # --- Serial vs parallel across firings (vectorized in both) ---
    workers = min(args.workers, n_firings)
    if workers > 1:
        def run_parallel():
            return run_parallel_plume_strikes(
                jfh_steps=steps,
                face_centroids=centroids,
                target_unit_normals=normals,
                thruster_data=vv.thruster_data,
                thruster_metrics=getattr(vv, 'thruster_metrics', None),
                plume_params=extract_plume_params(me),
                workers=workers,
            )

        t_par, par_results = time_best_of(run_parallel, args.repeat)
        print(f"parallel ({workers} workers): {t_par:8.3f} s"
              f"   ({t_vec / t_par:5.2f}x vs serial vectorized; includes "
              f"process startup)")
        for vec, par in zip(vec_results, par_results):
            assert np.array_equal(vec['strikes'], par['strikes']), \
                "serial/parallel strike mismatch"
    else:
        print(f"parallel benchmark skipped (workers={workers})")


if __name__ == '__main__':
    main()
