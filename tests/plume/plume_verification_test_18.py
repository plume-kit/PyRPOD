# ========================
# PyRPOD: tests/plume/plume_verification_test_18.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 16: normalized radial velocity Vr
# contours, Kn = 100, S0 = 2.0. Analytical field in the upper
# half-plane; lower half-plane reserved for the DSMC digitized overlay.
# Analytic content is Kn-independent; Figs. 16-18 differ only in their
# DSMC data.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_18.py
# Digitized overlays looked for: fig16_*.csv.

import plume_figure_utils as u

VR_LEVELS = [1.0, 1.8, 2.0, 2.2, 2.4, 2.8]


def generate_figure():
    return u.top_bottom_contour_figure(
        'Vr', VR_LEVELS, '100', 'fig16',
        'Normalized velocity $V_r$ distribution')


if __name__ == '__main__':
    u.run_script(generate_figure)
