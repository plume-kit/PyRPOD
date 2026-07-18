# ========================
# PyRPOD: tests/plume/plume_verification_error_summary.py
# ========================
# Model-vs-model analog of Cai & Wang 2012 Table 1: maximum relative
# differences in density and mass flux between the analytical,
# simplified and Simons (exit-referenced, Boyton kappa) plume models
# along the centerline and along the angular curve r/D = 10, S0 = 2.0.
#
# Table 1's reference columns are DSMC results, which PyRPOD does not
# have yet; those columns are emitted as "pending digitized data" and
# will be filled from tests/plume/data/digitized/ in a follow-up. As
# the paper notes, "the largest density relative error actually
# happens within the curve tails with very small values" -- each row
# therefore also reports the restricted-range maximum.
#
# The filename intentionally avoids pytest's collection patterns
# (test_*.py / *_test_*.py): this is a manual-run generator --
#     python tests/plume/plume_verification_error_summary.py
# It writes tests/plume/output/model_error_summary.csv and .md.

import numpy as np

import plume_figure_utils as u

S_0 = 2.0


def _row(curve, quantity, comparison, values, reference, mask, mask_note):
    full = u.max_rel_diff(values, reference)
    restricted = u.max_rel_diff(values[mask], reference[mask])
    return {'curve': curve, 'quantity': quantity, 'comparison': comparison,
            'max_rel_diff': full,
            'max_rel_diff_restricted': restricted,
            'restriction': mask_note,
            'vs_DSMC': 'pending digitized data'}


def build_rows():
    rows = []

    # centerline, X/D in [X_MIN, 10]. As in the paper's Figs. 19-21,
    # "simplified" on the centerline is the Eq. 18 closed form (paper
    # Sec. II.B) and "analytical" is the Eq. 5 integral; they coincide
    # to quadrature accuracy.
    x_over_D = np.linspace(u.X_MIN_OVER_D, 10.0, 200)
    r = x_over_D * u.D_NOZZLE
    n_analytical = u.full_field_values('n', r, np.zeros_like(r), S_0)
    n_simplified = u.centerline_profiles(x_over_D, S_0, ('n',))['n']
    n_simons = u.simons_density_field(r, np.zeros_like(r), S_0)
    core = x_over_D >= 1.0
    rows.append(_row('centerline', 'density',
                     'simplified (Eq. 18) vs analytical (Eq. 5)',
                     n_simplified, n_analytical, core, 'X/D >= 1'))
    rows.append(_row('centerline', 'density', 'Simons vs analytical',
                     n_simons, n_analytical, core, 'X/D >= 1'))

    # angular curve r/D = 10
    theta = np.deg2rad(np.linspace(0.0, u.THETA_MAX_DEG, 90))
    X = 10.0 * u.D_NOZZLE * np.cos(theta)
    Z = 10.0 * u.D_NOZZLE * np.sin(theta)
    nA = u.full_field_values('n', X, Z, S_0)
    Q = X ** 2 / (X ** 2 + Z ** 2)
    nS = u.get_K_factor(Q, S_0) / (2 * np.sqrt(np.pi)) * (u.R_0 / X) ** 2
    nSim = u.simons_density_field(X, Z, S_0)
    VrA = u.full_field_values('Vr', X, Z, S_0)
    fluxA = nA * VrA
    UA = u.full_field_values('U', X, Z, S_0)
    WA = u.full_field_values('W', X, Z, S_0)
    # simplified-model flux with the same Fig. 25 normalization
    US = u.get_M_factor(Q, S_0) / u.get_K_factor(Q, S_0)
    WS = US * Z / X
    VrS = (X * US + Z * WS) / np.sqrt(X ** 2 + Z ** 2)
    fluxS = nS * VrS
    core = theta <= np.deg2rad(60.0)
    rows.append(_row('angular r/D=10', 'density', 'simplified vs analytical',
                     nS, nA, core, 'theta <= 60 deg'))
    rows.append(_row('angular r/D=10', 'density', 'Simons vs analytical',
                     nSim, nA, core, 'theta <= 60 deg'))
    rows.append(_row('angular r/D=10', 'mass flux', 'simplified vs analytical',
                     fluxS, fluxA, core, 'theta <= 60 deg'))
    return rows


def write_summary():
    rows = build_rows()
    u.OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    columns = ['curve', 'quantity', 'comparison', 'max_rel_diff',
               'max_rel_diff_restricted', 'restriction', 'vs_DSMC']

    csv_path = u.OUTPUT_DIR / 'model_error_summary.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(','.join(columns) + '\n')
        for row in rows:
            fh.write(','.join(
                f'{row[c]:.4g}' if isinstance(row[c], float) else str(row[c])
                for c in columns) + '\n')

    md_path = u.OUTPUT_DIR / 'model_error_summary.md'
    with open(md_path, 'w', encoding='utf-8', newline='') as fh:
        fh.write('# Model-vs-model error summary (Table 1 analog)\n\n')
        fh.write(f'Cai & Wang 2012 conditions, S0 = {S_0}. DSMC reference '
                 'columns are pending digitized data.\n\n')
        fh.write('| ' + ' | '.join(columns) + ' |\n')
        fh.write('|' + '---|' * len(columns) + '\n')
        for row in rows:
            fh.write('| ' + ' | '.join(
                f'{row[c]:.4g}' if isinstance(row[c], float) else str(row[c])
                for c in columns) + ' |\n')
        fh.write('\nNote: unrestricted maxima occur in the plume tails '
                 'where the reference density is very small (the paper '
                 'makes the same observation about Table 1).\n')
    return csv_path, md_path


if __name__ == '__main__':
    for path in write_summary():
        print(f'wrote {path}')
