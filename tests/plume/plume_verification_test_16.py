# ========================
# PyRPOD: tests/plume/plume_verification_test_16.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 14: normalized U-velocity contours
# U1*sqrt(beta0), Kn = 100, S0 = 2.0. Analytical field in the upper
# half-plane; lower half-plane reserved for the DSMC digitized overlay.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_16.py
# Digitized overlays looked for: fig14_*.csv.

import plume_figure_utils as u


def generate_figure():
    return u.top_bottom_contour_figure(
        'U', [0.6, 1.0, 1.8, 2.4], '100', 'fig14',
        'Normalized U-velocity distribution')


if __name__ == '__main__':
    u.run_script(generate_figure)
