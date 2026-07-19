# ========================
# PyRPOD: tests/plume/plume_impingement_error_summary.py
# ========================
# Extends the model-vs-model error-summary pattern of
# plume_verification_error_summary.py to the Cai 2016 inclined-plate
# impingement study (Figs. 17-21 conditions): maximum and mean relative
# differences between the exact reference surface coefficients
# (pyrpod/plume/CaiImpingement2016.py, Eqs. 9-14) and the current
# PyRPOD approximation (SimplifiedGasKinetics field + Shen/Maxwellian
# wall formulas at the true incidence angle -- the Phase-2-fixed strike
# pipeline chain), over the plate contour grid.
#
# As in the 2012 summary, each row restricts the comparison to the
# region where the reference magnitude is at least 5% of its peak: the
# largest relative differences otherwise occur near the plate corners
# and sign-change lines where the reference value is near zero.
# DSMC reference columns are pending digitized data
# (tests/plume/data/digitized/fig17_*.csv ... fig21_*.csv).
#
# The filename intentionally avoids pytest's collection patterns
# (test_*.py / *_test_*.py): this is a manual-run generator --
#     python tests/plume/plume_impingement_error_summary.py
# It writes tests/plume/output/cai2016_error_summary.csv and .md.

import numpy as np

import plume_impingement_utils as u
from pyrpod.plume import CaiImpingement2016 as cai


def build_rows():
    # module self-verification first: quadrature convergence, symmetry,
    # specular identity, and the n_w flux balance (raises on failure)
    cai.run_sanity_checks(u.S_0, u.ALPHA_0, u.EPS, u.R_0, u.L_PLATE,
                          verbose=False)

    ref = u.reference_grid()
    chain_d = u.chain_grid()            # diffuse chain (sigma = 1)
    chain_s = u.chain_grid(sigma=0.0)   # specular chain for Cp,s

    ref_shear = np.hypot(ref['Cf1_d'], ref['Cf2_d'])
    comparisons = [
        ('Cp,d (Fig. 17)', chain_d['Cp'], ref['Cp_d']),
        ('Cp,s (Fig. 18)', chain_s['Cp'], ref['Cp_s']),
        ('Cf1,d (Fig. 19)', chain_d['Cf1'], ref['Cf1_d']),
        ('Cf2,d (Fig. 20)', chain_d['Cf2'], ref['Cf2_d']),
        ('|Cf,d| (shear magnitude)', chain_d['Cshear'], ref_shear),
        ('Cq,d (Fig. 21)', chain_d['Cq'], ref['Cq_d']),
    ]

    rows = []
    for quantity, chain, reference in comparisons:
        mask = u.significant_mask(reference)
        rows.append({
            'quantity': quantity,
            'comparison': 'PyRPOD chain vs Cai 2016 exact',
            'max_rel_diff': u.base.max_rel_diff(chain[mask],
                                                reference[mask]),
            'mean_rel_diff': u.mean_rel_diff(chain, reference, mask),
            'restriction': '|ref| >= 5% of peak',
            'vs_DSMC': 'pending digitized data',
        })
    return rows


def write_summary():
    rows = build_rows()
    u.base.OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    columns = ['quantity', 'comparison', 'max_rel_diff', 'mean_rel_diff',
               'restriction', 'vs_DSMC']

    csv_path = u.base.OUTPUT_DIR / 'cai2016_error_summary.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(','.join(columns) + '\n')
        for row in rows:
            fh.write(','.join(
                f'{row[c]:.4g}' if isinstance(row[c], float) else str(row[c])
                for c in columns) + '\n')

    md_path = u.base.OUTPUT_DIR / 'cai2016_error_summary.md'
    with open(md_path, 'w', encoding='utf-8', newline='') as fh:
        fh.write('# Cai 2016 impingement error summary '
                 '(reference vs PyRPOD chain)\n\n')
        fh.write('Section-4 conditions: argon, D = 1 m, S0 = 2.0, '
                 'L = 4D, alpha0 = 60 deg, Tw/T0 = 1.5, 8 m x 8 m '
                 'plate, contour grid '
                 f'{u.GRID_N}x{u.GRID_N}. DSMC reference columns are '
                 'pending digitized data.\n\n')
        fh.write('| ' + ' | '.join(columns) + ' |\n')
        fh.write('|' + '---|' * len(columns) + '\n')
        for row in rows:
            fh.write('| ' + ' | '.join(
                f'{row[c]:.4g}' if isinstance(row[c], float) else str(row[c])
                for c in columns) + ' |\n')
        fh.write('\nNotes: the PyRPOD chain is the strike-pipeline '
                 'computation (SimplifiedGasKinetics local field + '
                 'Shen/Maxwellian wall model at the true incidence '
                 'angle); Cp,s uses sigma = 0. Rows are restricted to '
                 '|ref| >= 5% of the peak magnitude -- unrestricted '
                 'maxima occur where the reference is near zero (plate '
                 'corners, stagnation/sign-change lines), the same '
                 'caveat as the 2012 summary.\n')
    return csv_path, md_path


if __name__ == '__main__':
    for path in write_summary():
        print(f'wrote {path}')
