# ========================
# PyRPOD: tests/rpod/rpod_integration_test_07.py
# ========================
# Cai 2016 inclined-plate verification: runs case/plume/plume_inclined_plate
# (argon round jet, D = 1 m, S0 = 2.0, 8 m x 8 m plate at L = 4D inclined
# 60 deg, Tw/T0 = 1.5, sigma = 1) through PlumeStrikeEstimationStudy,
# converts the per-face dimensional loads to the paper's coefficient
# normalization (pressure/shear by n0*m*U0^2/2, heat flux by n0*m*U0^3/2),
# and compares face-by-face against the exact reference functions
# (pyrpod/plume/CaiImpingement2016.py, Eqs. 9-13) evaluated at the face
# centroids.
#
# Hard assertions cover what must be exact:
#   - geometry: every plate face is struck exactly once (the whole plate
#     sits inside the gating wedge/radius at this pose);
#   - the vectorized and scalar strike paths agree (strike arrays exactly,
#     kinetics arrays to 1e-12, per the plume_unit_test_04 convention);
#   - internal consistency: the pipeline loads match the analytic
#     PyRPOD-approximation chain of tests/plume/plume_impingement_utils
#     (same SimplifiedGasKinetics field + Shen/Maxwellian wall model at the
#     true incidence angle) to <2% -- the only difference is the hit test's
#     legacy 3.14-based positional theta feeding the field evaluation.
#
# The pipeline-vs-reference gap itself is QUANTITATIVE DOCUMENTATION, not a
# hard tolerance gate: the Maxwellian chain is an engineering approximation
# of Cai's exact solution, and the deliverable is the measured gap. Max and
# mean relative errors (restricted to faces where the reference magnitude
# is >= 5% of its peak, as in the error summaries) are printed and appended
# to tests/plume/output/cai2016_pipeline_error_summary.md.
#
# Run:  python -m pytest rpod/rpod_integration_test_07.py -s   (from tests/)

import sys
import time
import unittest
from pathlib import Path

import numpy as np

_TESTS_DIR = Path(__file__).resolve().parents[1]
if str(_TESTS_DIR / 'plume') not in sys.path:
    sys.path.insert(0, str(_TESTS_DIR / 'plume'))
if str(_TESTS_DIR.parent) not in sys.path:  # repo root for direct runs
    sys.path.insert(0, str(_TESTS_DIR.parent))

import plume_impingement_utils as piu  # noqa: E402
from pyrpod.mission import MissionEnvironment  # noqa: E402
from pyrpod.plume import CaiImpingement2016 as cai  # noqa: E402
from pyrpod.plume.PlumeStrikeCalculator import (  # noqa: E402
    _compute_plume_strikes_scalar,
    compute_face_centroids,
    compute_plume_strikes,
)
from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy  # noqa: E402
from pyrpod.vehicle import TargetVehicle, VisitingVehicle  # noqa: E402

CASE_DIR = '../case/plume/plume_inclined_plate/'
SUMMARY_PATH = _TESTS_DIR / 'plume' / 'output' / 'Cai2016' / \
    'cai2016_pipeline_error_summary.md'


def face_areas(vectors):
    v0, v1, v2 = vectors[:, 0], vectors[:, 1], vectors[:, 2]
    return 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)


class InclinedPlateVerificationChecks(unittest.TestCase):

    def test_pipeline_vs_cai2016_reference(self):
        # 1. Set up (driver pattern of rpod_integration_test_05)
        jfh = JetFiringHistory.JetFiringHistory(CASE_DIR)
        jfh.read_jfh()

        tv = TargetVehicle.TargetVehicle(CASE_DIR)
        tv.set_stl()

        vv = VisitingVehicle.VisitingVehicle(CASE_DIR)
        vv.set_thruster_config()
        vv.set_thruster_metrics()

        me = MissionEnvironment.MissionEnvironment(CASE_DIR)

        study = PlumeStrikeEstimationStudy.PlumeStrikeEstimationStudy(me)
        study.study_init(jfh, tv, vv)

        # 2. Execute the full pipeline (writes VTK to the case results dir)
        t0 = time.perf_counter()
        firing_data = study.jfh_plume_strikes()
        t_pipeline = time.perf_counter() - t0

        self.assertEqual(list(firing_data.keys()), ['1'])
        result = firing_data['1']

        target = tv.mesh
        n_faces = len(target.vectors)
        centroids = compute_face_centroids(target.vectors)
        normals = target.get_unit_normals()

        # 3. Geometry: the whole plate lies inside the gating wedge/radius
        # at the paper pose, so every face must be struck exactly once.
        np.testing.assert_array_equal(result['strikes'], np.ones(n_faces))

        # 4. Vectorized and scalar paths must agree (shared incidence fix).
        step = {'thrusters': jfh.JFH[0]['thrusters'],
                'xyz': np.array(jfh.JFH[0]['xyz']),
                'dcm': np.array(jfh.JFH[0]['dcm']),
                't': float(jfh.JFH[0]['t'])}
        vec = compute_plume_strikes(target, normals, vv, step, me,
                                    face_centroids=centroids)
        scal = _compute_plume_strikes_scalar(target, normals, vv, step, me)
        np.testing.assert_array_equal(vec['strikes'], scal['strikes'])
        for key in ('pressures', 'shear_stress', 'heat_flux_rate'):
            self.assertTrue(np.allclose(vec[key], scal[key],
                                        rtol=1e-12, atol=1e-12),
                            msg=f'{key}: scalar vs vectorized differ')

        # 5. Convert to the paper's coefficients. Thruster at the origin
        # firing +X, so centroid coordinates are the paper's (X, Y, Z).
        Cp_pipe = result['pressures'] / piu.Q_DYN
        Csh_pipe = result['shear_stress'] / piu.Q_DYN
        Cq_pipe = result['heat_flux_rate'] / piu.Q_DYN_HEAT
        # firing time is 1 s, so the load equals the rate
        self.assertTrue(np.allclose(result['heat_flux_load'],
                                    result['heat_flux_rate']))

        # 6. Exact reference at the face centroids.
        ref = cai.surface_coefficients(
            centroids[:, 0], centroids[:, 1], centroids[:, 2],
            piu.S_0, piu.ALPHA_0, piu.EPS, piu.R_0)
        ref_shear = np.hypot(ref['Cf1_d'], ref['Cf2_d'])

        # 7. Internal consistency vs the Phase-1 analytic chain on a face
        # subsample (every 97th face covers the plate quasi-uniformly).
        # Denominators are floored at 5% of the subsample peak: the
        # Maxwellian heat flux changes sign on the plate (adiabatic-wall
        # crossover), so a purely relative test would divide by ~0 there
        # while the absolute agreement stays tight.
        sub = np.arange(0, n_faces, 97)
        chain = np.array([piu.chain_point_loads(*centroids[i]) for i in sub])
        for pipe_vals, chain_vals, name in [
                (result['pressures'][sub], chain[:, 0], 'pressure'),
                (result['shear_stress'][sub], chain[:, 1], 'shear'),
                (result['heat_flux_rate'][sub], chain[:, 2], 'heat flux')]:
            scale = np.maximum(np.abs(chain_vals),
                               0.05 * np.max(np.abs(chain_vals)))
            rel = np.abs(pipe_vals - chain_vals) / scale
            self.assertLess(np.max(rel), 2e-2,
                            msg=f'pipeline diverged from analytic chain '
                                f'({name}): max rel {np.max(rel):.3g}')

        # 8. Quantitative documentation of the pipeline-vs-reference gap.
        areas = face_areas(target.vectors)
        rows = []
        for name, pipe, reference in [
                ('Cp,d', Cp_pipe, ref['Cp_d']),
                ('|Cf,d|', Csh_pipe, ref_shear),
                ('Cq,d', Cq_pipe, ref['Cq_d'])]:
            mask = piu.significant_mask(reference)
            rel = np.abs(pipe[mask] - reference[mask]) / np.abs(reference[mask])
            rows.append((name, float(np.max(rel)), float(np.mean(rel)),
                         float(np.sum(pipe * areas) / np.sum(areas)),
                         float(np.sum(reference * areas) / np.sum(areas))))
            print(f'[cai2016] {name}: max rel err {np.max(rel):.3f}, '
                  f'mean rel err {np.mean(rel):.3f} '
                  f'(over {mask.sum()}/{n_faces} significant faces)')
        print(f'[cai2016] pipeline wall time: {t_pipeline:.1f} s '
              f'({n_faces} faces, 1 firing)')

        SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(SUMMARY_PATH, 'w', encoding='utf-8', newline='') as fh:
            fh.write('# Cai 2016 pipeline error summary '
                     '(strike pipeline vs exact reference)\n\n')
            fh.write('Written by tests/rpod/rpod_integration_test_07.py: '
                     'face-by-face comparison of the '
                     'PlumeStrikeEstimationStudy loads (Simplified '
                     'kinetics + Maxwellian wall model at the true '
                     'incidence angle) against '
                     'pyrpod/plume/CaiImpingement2016.py at the paper '
                     f'conditions; {n_faces} faces, pipeline wall time '
                     f'{t_pipeline:.1f} s.\n\n')
            fh.write('| quantity | max_rel_err | mean_rel_err | '
                     'plate avg (pipeline) | plate avg (reference) |\n')
            fh.write('|---|---|---|---|---|\n')
            for name, mx, mn, avg_p, avg_r in rows:
                fh.write(f'| {name} | {mx:.4g} | {mn:.4g} | {avg_p:.4g} | '
                         f'{avg_r:.4g} |\n')
            fh.write('\nRelative errors restricted to faces with '
                     '|reference| >= 5% of its peak (the same near-zero '
                     'caveat as the other summaries). The gap is the '
                     'documented accuracy of the Maxwellian engineering '
                     'chain vs the exact collisionless solution -- not a '
                     'regression gate.\n')
        print(f'[cai2016] wrote {SUMMARY_PATH}')

        # Loose envelope so a future regression that breaks the chain
        # entirely (e.g. reverting to positional theta) fails loudly:
        # with the orientation-blind bug, mean Cp error was ~3x larger.
        self.assertLess(rows[0][2], 0.15,
                        msg='mean Cp error vs reference far above the '
                            'documented Maxwellian-chain gap')


if __name__ == '__main__':
    unittest.main()
