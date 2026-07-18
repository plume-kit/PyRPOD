# ========================
# PyRPOD: tests/plume/plume_verification_test_15.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 13: normalized temperature contours
# T1/T0, Kn = 100, S0 = 2.0; X/D in [0, 2.5], Y/D in [-2.5, 2.5].
# Analytical field in the upper half-plane; lower half-plane reserved
# for the DSMC digitized overlay. This is the figure the paper uses to
# argue p = n*k*T0 is invalid (T1 < T0 everywhere downstream).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_15.py
# Digitized overlays looked for: fig13_*.csv.

import plume_figure_utils as u


def generate_figure():
    return u.top_bottom_contour_figure(
        'T', [0.3, 0.4, 0.6, 0.9], '100', 'fig13',
        'Normalized temperature distribution',
        x_max=2.5, z_max=2.5, z_axis_label='Y/D')


if __name__ == '__main__':
    u.run_script(generate_figure)
