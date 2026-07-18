# ========================
# PyRPOD: tests/plume/plume_verification_test_17.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 15: normalized "V-velocity" contours,
# Kn = 100, S0 = 2.0. In the plotted XOZ plane (Y = 0) the y-component
# V is identically zero by axisymmetry; the transverse component shown
# in the paper's Fig. 15 corresponds to the model's W (Eq. 7), which is
# what is plotted here. Analytical field in the upper half-plane; lower
# half-plane reserved for the DSMC digitized overlay.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_17.py
# Digitized overlays looked for: fig15_*.csv.

import plume_figure_utils as u


def generate_figure():
    return u.top_bottom_contour_figure(
        'W', [0.1, 0.5, 1.0], '100', 'fig15',
        'Normalized V-velocity distribution (transverse W at Y = 0)')


if __name__ == '__main__':
    u.run_script(generate_figure)
