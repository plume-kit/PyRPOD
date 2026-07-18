# ========================
# PyRPOD: tests/plume/plume_verification_test_09.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 7: normalized number-density contours,
# Kn = 0.1, S0 = 2.0. Same analytic content as Fig. 6 (collisionless
# models are Kn-independent); the Kn distinction lives in the DSMC
# overlay slot.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_09.py
# Digitized overlays looked for: fig07_*.csv (one CSV per digitized
# DSMC contour polyline, lower half-plane).

import plume_figure_utils as u


def generate_figure():
    return u.density_contour_figure('0.1', 'fig07')


if __name__ == '__main__':
    u.run_script(generate_figure)
