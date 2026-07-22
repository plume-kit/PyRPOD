# ========================
# PyRPOD: tests/rpod/rpod_verification_test_06.py
# ========================
# Cai 2016 flat-plate SWEEP verification: the swept analog of
# rpod_integration_test_07. Runs the 95-firing sweep JFH of
# case/plume/plume_flat_plate_sweep (a flat alpha0 = 0 reframing of the
# paper geometry; VV on arcs of radius L about the stationary plate,
# thruster aimed at the plate center; 19 approach angles alpha = -90..90 deg
# x 5 stand-off distances L/D in {2,4,6,8,10}) through the strike pipeline's
# per-firing core (compute_plume_strikes -- the same function
# PlumeStrikeEstimationStudy calls per firing), reduces each pose to the
# Eq.-15 plate-averaged coefficients, and compares them against the exact
# reference (pyrpod/plume/CaiImpingement2016.py). The 60 deg inclined sweep
# lives in the sibling case plume_inclined_plate_sweep (driven by the study
# script); the reframing is physics-invariant, so both give identical
# coefficients -- flat just reads more cleanly in ParaView.
#
# This is the pytest promotion of the assertion-relevant core of
# scripts/inclined_plate_sweep_study.py; that script is kept intact as the
# full-artifact generator (CSV, coefficient/peak plots, per-pose VTK +
# ParaView collections). Only the invariants live here -- no plots/CSV.
# Per-pose strike VTK (one .vtu per pose + ParaView .pvd collections, for
# visual inspection) is OPT-IN: set the env var PYRPOD_SWEEP_VTK=1 to write
# them to the case's results/sweep/ (gitignored); the default run is lean.
#
# Hard gates (a paper-frame convention error breaks these as O(1)):
#   - the sweep JFH has exactly len(ALPHAS) x len(L/D) = 95 firings;
#   - mirror symmetry: results in +/-alpha match to <1e-5 relative (the
#     JFH stores DCMs to 6 significant digits, so mirrored poses reproduce
#     to ~1e-6); the edge-on +/-90 deg poses are excluded (their strike
#     membership is float32-epsilon-arbitrary, see the script docstring);
#   - CF2 vanishes identically by symmetry (~0 to 1e-4 of the CP scale).
#
# The pipeline-vs-reference gap is QUANTITATIVE DOCUMENTATION with a loose
# regression gate (as in rpod_integration_test_07): the Maxwellian chain
# is an engineering approximation of Cai's exact solution. Plate-averaged
# CP/CF1/CQ max/mean relative errors (non-edge-on poses) are printed and
# written to tests/plume/output/Cai2016/cai2016_sweep_error_summary.md.
#
# Run:  python -m pytest rpod/rpod_verification_test_06.py -s   (from tests/)
# Inspect: PYRPOD_SWEEP_VTK=1 python -m pytest rpod/rpod_verification_test_06.py -s
#          (writes the per-pose strike .vtu files + sweep_LoD*.pvd)

import os
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
    compute_face_centroids,
    compute_plume_strikes,
)
from pyrpod.rpod import JetFiringHistory  # noqa: E402
from pyrpod.util.stl.stl import convert_stl_to_vtk  # noqa: E402
from pyrpod.vehicle import TargetVehicle, VisitingVehicle  # noqa: E402

# Dedicated flat-plate sweep case (alpha0 = 0, plate center at the origin):
# a pure global-frame reframing of the paper geometry so the swept strikes
# read cleanly in ParaView (flat plate in the X-Y plane, VV swept above it).
# Its config.ini selects the flat STL (flat_plate_transformed.stl) and the
# flat sweep JFH (jfh_flat_plate_sweep.A), so no runtime override is needed;
# the ALPHA0_DEG/PLATE_CENTER constants below match those assets (generated
# by stl/transform_inclined_plate.py --alpha0-deg 0 --distance 0 and
# jfh/generate_sweep_jfh.py --alpha0-deg 0 --distance 0). The paper single
# pose lives in plume_inclined_plate (rpod_integration_test_07); the 60 deg
# sweep in plume_inclined_plate_sweep (the study script).
CASE_DIR = '../case/plume/plume_flat_plate_sweep/'
SUMMARY_PATH = _TESTS_DIR / 'plume' / 'output' / 'Cai2016' / \
    'cai2016_sweep_error_summary.md'

# Per-pose strike VTK export (opt-in for inspection in ParaView; off by
# default so the normal pytest run stays lean). Enable by setting the
# environment variable PYRPOD_SWEEP_VTK=1. Strikes follow the standard RPOD
# pipeline convention -- one .vtu per firing, numbered by JFH index:
#   results/strikes/firing-<i>.vtu   (i = 0 .. 94, one pose each)
#   results/strikes/sweep_LoD<LD>.pvd  (ParaView collections; alpha as time)
WRITE_VTK = bool(os.environ.get('PYRPOD_SWEEP_VTK'))
_CASE_ROOT = _TESTS_DIR.parent / 'case' / 'plume' / 'plume_flat_plate_sweep'
STRIKES_DIR = _CASE_ROOT / 'results' / 'strikes'

# Sweep definition (mirrors scripts/inclined_plate_sweep_study.py).
PLATE_CENTER = np.array([0.0, 0.0, 0.0])
ALPHA0_DEG = 0.0                       # the mesh's fixed global tilt
ALPHAS_DEG = np.arange(-90.0, 90.0 + 1e-9, 10.0)
L_OVER_D = [2.0, 4.0, 6.0, 8.0, 10.0]
COEFF_NAMES = ['CP', 'CF1', 'CF2', 'CQ', 'CM', 's_cc']

_A0 = np.deg2rad(ALPHA0_DEG)
NORMAL_OUT = np.array([-np.sin(_A0), 0.0, np.cos(_A0)])  # toward the VV side
TANGENT_TAU = np.array([np.cos(_A0), 0.0, np.sin(_A0)])
TANGENT_S = np.array([0.0, 1.0, 0.0])


def face_areas(vectors):
    v0, v1, v2 = vectors[:, 0], vectors[:, 1], vectors[:, 2]
    return 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)


def pipeline_coeffs(result, centroids, areas, tau_face, thruster_pos,
                    alpha_deg):
    """Eq.-15 averaged coefficients, per-face peaks, and the case-frame
    per-face field arrays (for VTK) for one firing.

    Faithful to scripts/inclined_plate_sweep_study.py's pipeline_row.
    Signed shear components are decomposed onto the plate (tau, s) axes
    along the tangential projection of the radial flow direction; alpha > 0
    poses are sign-flipped (CF1/CM) into the paper convention. The returned
    face_fields carry the un-flipped case-frame components, matching the
    actual mesh (as the script's VTK export does).
    """
    S_tot = float(np.sum(areas))
    Cp = result['pressures'] / piu.Q_DYN
    Csh = result['shear_stress'] / piu.Q_DYN
    Cq = result['heat_flux_rate'] / piu.Q_DYN_HEAT

    rel = centroids - thruster_pos
    dist = np.linalg.norm(rel, axis=1)
    u_hat = rel / dist[:, None]
    n_in = -NORMAL_OUT
    cos_inc = u_hat @ n_in
    t_vec = u_hat - cos_inc[:, None] * n_in
    t_norm = np.linalg.norm(t_vec, axis=1)
    safe = t_norm > 1e-12
    t_hat = np.zeros_like(t_vec)
    t_hat[safe] = t_vec[safe] / t_norm[safe, None]
    Cf1 = Csh * (t_hat @ TANGENT_TAU)
    Cf2 = Csh * (t_hat @ TANGENT_S)

    flip = -1.0 if alpha_deg > 0 else 1.0   # into the paper convention
    CP = float(np.sum(Cp * areas) / S_tot)
    CF1 = flip * float(np.sum(Cf1 * areas) / S_tot)
    CF2 = float(np.sum(Cf2 * areas) / S_tot)
    CQ = float(np.sum(Cq * areas) / S_tot)
    CM = flip * float(np.sum(tau_face * Cp * areas) / (2.0 * piu.H_0 * S_tot))
    s_cc = CM / CP if CP != 0.0 else float('nan')

    face_fields = {
        'strikes': result['strikes'],
        'pressure_Pa': result['pressures'],
        'shear_Pa': result['shear_stress'],
        'heat_flux_W_m2': result['heat_flux_rate'],
        'Cp': Cp,
        'Cshear': Csh,
        'Cf1_case': Cf1,
        'Cf2_case': Cf2,
        'Cq': Cq,
    }
    return ({'CP': CP, 'CF1': CF1, 'CF2': CF2, 'CQ': CQ, 'CM': CM,
             's_cc': s_cc},
            {'peak_Cp': float(np.max(Cp)), 'peak_Cshear': float(np.max(Csh)),
             'peak_Cq': float(np.max(Cq)),
             'n_struck': int(np.count_nonzero(result['strikes']))},
            face_fields)


def write_pose_vtk(target, face_fields, vtk_dir, base_name):
    """Write one pose's per-face fields to vtk_dir/<base_name>.vtu via the
    shared convert_stl_to_vtk writer (pyevtk needs C-contiguous float64)."""
    cell_data = {k: np.ascontiguousarray(v, dtype=np.float64)
                 for k, v in face_fields.items()}
    convert_stl_to_vtk(target, vtk_dir, filename=base_name,
                       cellData=cell_data)


def write_pvd(path, entries):
    """ParaView collection grouping (timestep, relative_vtu_path) entries."""
    lines = ['<?xml version="1.0"?>',
             '<VTKFile type="Collection" version="0.1" '
             'byte_order="LittleEndian">', '  <Collection>']
    for timestep, rel_file in sorted(entries):
        lines.append(f'    <DataSet timestep="{timestep:g}" group="" '
                     f'part="0" file="{rel_file}"/>')
    lines += ['  </Collection>', '</VTKFile>']
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def reference_rows():
    """Exact Eq.-15 coefficients per (|alpha|, L), cached (mirror-invariant)."""
    cache = {}
    for L in L_OVER_D:
        for abs_alpha in sorted({abs(a) for a in ALPHAS_DEG}):
            alpha_paper = np.deg2rad(90.0 - abs_alpha)
            cache[(abs_alpha, L)] = cai.averaged_coefficients(
                piu.S_0, alpha_paper, piu.EPS, piu.R_0, L, piu.W_0, piu.H_0)
    return cache


def check_mirror_symmetry(rows):
    """Results must be mirror-symmetric in +/-alpha (paper convention).

    The +/-90 deg poses are excluded: their strike membership comes from
    float32-epsilon signs in the facing test, which are not mirrored.
    CF2 is checked separately -- it vanishes identically by symmetry, so
    normalizing its asymmetry by its own (noise) scale is meaningless.
    """
    worst = 0.0
    for L in L_OVER_D:
        by_alpha = {r['alpha_deg']: r for r in rows if r['L_over_D'] == L
                    and abs(r['alpha_deg']) < 90.0}
        cp_scale = max(abs(r['CP_pipe']) for r in by_alpha.values())
        # tolerance covers the triangulation's O(h^2) s-asymmetry (the
        # cell-diagonal split is not mirror-invariant)
        assert max(abs(r['CF2_pipe']) for r in by_alpha.values()) \
            < 1e-4 * cp_scale, f'CF2 not ~0 at L/D={L}'
        for a in ALPHAS_DEG[(ALPHAS_DEG > 0) & (ALPHAS_DEG < 90.0)]:
            for name in ('CP', 'CF1', 'CQ', 'CM'):
                lo, hi = by_alpha[-a][f'{name}_pipe'], by_alpha[a][f'{name}_pipe']
                scale = max(abs(v) for r in by_alpha.values()
                            for v in [r[f'{name}_pipe']])
                scale = max(scale, 1e-6 * cp_scale)
                worst = max(worst, abs(hi - lo) / scale)
    # tolerance: the JFH file stores DCMs to 6 significant digits (and
    # positions to 9), so mirrored poses reproduce to ~1e-6 relative;
    # a convention error would show up as O(1).
    assert worst < 1e-5, f'mirror symmetry violated: {worst:.3g}'
    return worst


def reference_envelope(rows):
    """Plate-averaged pipeline-vs-reference relative error over the
    non-edge-on poses (|alpha| < 90). Denominators are floored at 5% of
    each coefficient's own reference peak so the near-zero angles do not
    divide by ~0 (the significant_mask convention). Returns
    {name: (max_rel, mean_rel, n_poses)} for CP/CF1/CQ."""
    sub = [r for r in rows if abs(r['alpha_deg']) < 90.0]
    out = {}
    for name in ('CP', 'CF1', 'CQ'):
        pipe = np.array([r[f'{name}_pipe'] for r in sub])
        ref = np.array([r[f'{name}_ref'] for r in sub])
        scale = np.maximum(np.abs(ref), 0.05 * np.max(np.abs(ref)))
        rel = np.abs(pipe - ref) / scale
        out[name] = (float(np.max(rel)), float(np.mean(rel)), len(sub))
    return out


class InclinedPlateSweepVerification(unittest.TestCase):

    def test_sweep_vs_cai2016_reference(self):
        # 1. Set up the flat-sweep case; its config.ini already selects the
        # flat STL + 95-firing angle x distance JFH, so no override is needed.
        jfh = JetFiringHistory.JetFiringHistory(CASE_DIR)
        jfh.read_jfh()

        tv = TargetVehicle.TargetVehicle(CASE_DIR)
        tv.set_stl()

        vv = VisitingVehicle.VisitingVehicle(CASE_DIR)
        vv.set_thruster_config()
        vv.set_thruster_metrics()

        me = MissionEnvironment.MissionEnvironment(CASE_DIR)

        n_firings = len(jfh.JFH)
        self.assertEqual(
            n_firings, len(ALPHAS_DEG) * len(L_OVER_D),
            msg=f'unexpected sweep JFH length {n_firings}; regenerate with '
                'case/plume/plume_flat_plate_sweep/jfh/generate_sweep_jfh.py '
                '--alpha0-deg 0 --distance 0 --out jfh_flat_plate_sweep.A')

        # 2. Precompute the plate geometry once (poses share one mesh).
        target = tv.mesh
        n_faces = len(target.vectors)
        normals = target.get_unit_normals()
        centroids = compute_face_centroids(target.vectors)
        areas = face_areas(target.vectors)
        tau_face = (centroids - PLATE_CENTER) @ TANGENT_TAU

        # 3. Exact reference coefficients, cached per (|alpha|, L).
        t0 = time.perf_counter()
        ref_cache = reference_rows()
        t_ref = time.perf_counter() - t0

        # 4. Run every firing through the per-firing pipeline core. When
        # PYRPOD_SWEEP_VTK is set, also write each pose's per-face strikes
        # and loads to results/strikes/firing-<i>.vtu (one file per firing,
        # numbered by JFH index -- the standard RPOD strike convention;
        # independent poses, no cumulative accumulation).
        if WRITE_VTK:
            STRIKES_DIR.mkdir(parents=True, exist_ok=True)
        pvd_entries = {}   # L/D -> [(alpha_deg, relative vtu path)]

        rows = []
        t0 = time.perf_counter()
        for i in range(n_firings):
            L = L_OVER_D[i // len(ALPHAS_DEG)]
            alpha_deg = float(ALPHAS_DEG[i % len(ALPHAS_DEG)])
            step = {'thrusters': jfh.JFH[i]['thrusters'],
                    'xyz': np.array(jfh.JFH[i]['xyz']),
                    'dcm': np.array(jfh.JFH[i]['dcm']),
                    't': float(jfh.JFH[i]['t'])}
            result = compute_plume_strikes(target, normals, vv, step, me,
                                           face_centroids=centroids)
            coeffs, peaks, face_fields = pipeline_coeffs(
                result, centroids, areas, tau_face, step['xyz'], alpha_deg)
            ref = ref_cache[(abs(alpha_deg), L)]
            row = {'alpha_deg': alpha_deg, 'L_over_D': L, **peaks}
            for name in COEFF_NAMES:
                row[f'{name}_pipe'] = coeffs[name]
                row[f'{name}_ref'] = float(ref[name])
            rows.append(row)

            if WRITE_VTK:
                base = f'firing-{i}'   # strict JFH-index numbering
                write_pose_vtk(target, face_fields, STRIKES_DIR, base)
                pvd_entries.setdefault(L, []).append(
                    (alpha_deg, f'{base}.vtu'))
        t_sweep = time.perf_counter() - t0

        if WRITE_VTK:
            for L, entries in pvd_entries.items():
                write_pvd(STRIKES_DIR / f'sweep_LoD{int(L):02d}.pvd', entries)
            print(f'[cai2016-sweep] wrote {n_firings} per-pose strike VTK '
                  f'files to {STRIKES_DIR} (+ {len(pvd_entries)} '
                  f'sweep_LoD*.pvd collections)')

        # 5. Report the edge-on poses (excluded from the gates: strike
        # membership is epsilon-degenerate there, see the script docstring).
        head_on = {r['L_over_D']: r['CP_pipe'] for r in rows
                   if r['alpha_deg'] == 0.0}
        for r in rows:
            if abs(r['alpha_deg']) == 90.0:
                print(f"[cai2016-sweep] edge-on alpha={r['alpha_deg']:+.0f}, "
                      f"L/D={r['L_over_D']:g}: {r['n_struck']} faces pass the "
                      f"epsilon-degenerate facing test; CP={r['CP_pipe']:.3e} "
                      f"(head-on {head_on[r['L_over_D']]:.3e})")

        # 6. Hard convention gates.
        worst = check_mirror_symmetry(rows)
        self.assertLess(worst, 1e-5,
                        msg=f'mirror symmetry violated: {worst:.3g}')
        print(f'[cai2016-sweep] mirror-symmetry worst normalized asymmetry '
              f'{worst:.2e}')

        # 7. Quantitative documentation of the pipeline-vs-reference gap.
        env = reference_envelope(rows)
        for name in ('CP', 'CF1', 'CQ'):
            mx, mn, n = env[name]
            print(f'[cai2016-sweep] {name}: max rel err {mx:.3f}, '
                  f'mean rel err {mn:.3f} (over {n} non-edge-on poses)')
        print(f'[cai2016-sweep] timings: sweep {t_sweep:.1f} s '
              f'({t_sweep / n_firings * 1e3:.0f} ms/firing, {n_faces} faces), '
              f'reference {t_ref:.1f} s')

        SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(SUMMARY_PATH, 'w', encoding='utf-8', newline='') as fh:
            fh.write('# Cai 2016 inclined-plate sweep error summary '
                     '(strike pipeline vs exact reference)\n\n')
            fh.write('Written by tests/rpod/rpod_verification_test_06.py: '
                     'plate-averaged Eq.-15 coefficients from the '
                     'PlumeStrikeEstimationStudy per-firing core '
                     '(compute_plume_strikes; Simplified kinetics + '
                     'Maxwellian wall model at the true incidence angle) '
                     'over the 95-firing angle x distance sweep, compared '
                     'against pyrpod/plume/CaiImpingement2016.py '
                     f'(Eq. 15 quadrature). {len(ALPHAS_DEG)} approach '
                     f'angles x {len(L_OVER_D)} distances, {n_faces} faces, '
                     f'sweep wall time {t_sweep:.1f} s.\n\n')
            fh.write(f'Mirror-symmetry worst normalized asymmetry: '
                     f'{worst:.2e} (gate < 1e-5).\n\n')
            fh.write('| quantity | max_rel_err | mean_rel_err | poses |\n')
            fh.write('|---|---|---|---|\n')
            for name in ('CP', 'CF1', 'CQ'):
                mx, mn, n = env[name]
                fh.write(f'| {name} | {mx:.4g} | {mn:.4g} | {n} |\n')
            fh.write('\nPlate-averaged relative errors over the non-edge-on '
                     'poses (|alpha| < 90 deg), denominators floored at 5% '
                     "of each coefficient's reference peak. The gap is the "
                     'documented accuracy of the Maxwellian engineering '
                     'chain vs the exact collisionless solution -- not a '
                     'regression gate.\n')
        print(f'[cai2016-sweep] wrote {SUMMARY_PATH}')

        # 8. Loose envelope so a convention regression (e.g. a dropped
        # paper-frame sign flip) fails loudly while the normal
        # Maxwellian-chain gap passes. Calibrated from the measured run
        # (CP tracks reference ~1-3%, CF1 ~5%).
        self.assertLess(env['CP'][1], 0.08,
                        msg='mean plate-averaged CP error vs reference far '
                            'above the documented Maxwellian-chain gap')
        self.assertLess(env['CF1'][1], 0.15,
                        msg='mean plate-averaged CF1 error vs reference far '
                            'above the documented Maxwellian-chain gap')


if __name__ == '__main__':
    unittest.main()
