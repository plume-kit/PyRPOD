# ========================
# PyRPOD: tests/plume/plume_verification_test_28.py
# ========================
# Reproduces Cai 2016 Fig. 17: diffuse-plate surface pressure contours
# Cp,d(s, tau) for a round argon jet (D = 1 m, S0 = 2.0, T0 = 200 K) on
# the 8 m x 8 m plate at L = 4D, alpha0 = 60 deg, Tw/T0 = 1.5 (Eq. 9,
# solid black). Overlaid dashed red: the current PyRPOD approximation
# (SimplifiedGasKinetics field + Shen/Maxwellian wall pressure at the
# true incidence angle, sigma = 1) -- exactly what the fixed strike
# pipeline computes per struck face.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_28.py
# Digitized overlays looked for: cai16_fig17_*.csv (e.g.
# cai16_fig17_dsmc_0p2.csv, one CSV per digitized contour polyline; the
# cai16_ prefix keeps the 2016 paper's slots distinct from the Cai &
# Wang 2012 fig17 slot).

import plume_impingement_utils as u


def generate_figure():
    ref = u.reference_grid()
    chain = u.chain_grid()
    levels = [0.005, 0.01, 0.05, 0.1, 0.2]
    return u.impingement_contour_figure(
        'cai16_fig17', ref['Cp_d'], chain['Cp'], levels,
        'Diffuse plate $C_{p,d}(s,\\tau)$, $S_0$=2.0, '
        '$T_w/T_0$=1.5, $\\alpha_0$=60$^\\circ$ (Fig. 17)',
        'fig17_diffuse_pressure')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
