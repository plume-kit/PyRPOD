# ========================
# PyRPOD: tests/plume/plume_verification_test_10.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 8: normalized number-density contours,
# Kn = 0.01, S0 = 2.0. Same analytic content as Fig. 6 (collisionless
# models are Kn-independent); the Kn distinction lives in the DSMC
# overlay slot.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_10.py
# Digitized overlays looked for: fig08_*.csv (one CSV per digitized
# DSMC contour polyline, lower half-plane).

import plume_figure_utils as u


def generate_figure():
    return u.density_contour_figure('0.01', 'fig08')


if __name__ == '__main__':
    u.run_script(generate_figure)
