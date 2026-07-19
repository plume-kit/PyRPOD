# ========================
# PyRPOD: tests/plume/plume_verification_test_29.py
# ========================
# Reproduces Cai 2016 Fig. 18: specular-plate surface pressure contours
# Cp,s(s, tau) (Eq. 14, solid black) at the Section-4 conditions
# (argon, D = 1 m, S0 = 2.0, L = 4D, alpha0 = 60 deg). Overlaid dashed
# red: the current PyRPOD approximation with sigma = 0 -- Shen's
# Maxwellian pressure reduces to the fully specular reflection, and,
# consistent with the paper, the wall temperature drops out.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_29.py
# Digitized overlays looked for: fig18_*.csv (one CSV per digitized
# contour polyline).

import plume_impingement_utils as u


def generate_figure():
    ref = u.reference_grid()
    chain = u.chain_grid(sigma=0.0)
    levels = [0.005, 0.01, 0.05, 0.1, 0.2, 0.3]
    return u.impingement_contour_figure(
        'fig18', ref['Cp_s'], chain['Cp'], levels,
        'Specular plate $C_{p,s}(s,\\tau)$, $S_0$=2.0, '
        '$\\alpha_0$=60$^\\circ$ (Fig. 18)',
        'fig18_specular_pressure',
        chain_label='PyRPOD chain ($\\sigma$=0)')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
