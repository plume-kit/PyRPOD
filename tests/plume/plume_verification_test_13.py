# ========================
# PyRPOD: tests/plume/plume_verification_test_13.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 11: normalized pressure contours
# p1/p0 = (n1/n0)(T1/T0), Kn = 100, S0 = 2.0. Analytical field in the
# upper half-plane; lower half-plane reserved for the DSMC digitized
# overlay. Analytic content is Kn-independent; Figs. 11-12 differ only
# in their DSMC data.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_13.py
# Digitized overlays looked for: fig11_*.csv (one CSV per digitized
# DSMC contour polyline, lower half-plane).

import plume_figure_utils as u


def generate_figure():
    return u.top_bottom_contour_figure(
        'p', [1e-5, 1e-4, 1e-3, 1e-2], '100', 'fig11',
        'Normalized pressure contours')


if __name__ == '__main__':
    u.run_script(generate_figure)
