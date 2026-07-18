# ========================
# PyRPOD: tests/plume/plume_verification_test_08.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 6: normalized number-density contours,
# Kn = 100, S0 = 2.0. Upper half-plane: analytical (solid) + simplified
# (dashed); lower half-plane: Simons (dashed) + DSMC slot. The analytic
# curves are Kn-independent (collisionless); Figs. 6-8 differ only in
# their DSMC data, so each gets its own script/overlay slot.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_08.py
# Digitized overlays looked for: fig06_*.csv (e.g. fig06_dsmc_0p001.csv,
# one CSV per digitized DSMC contour polyline, lower half-plane).

import plume_figure_utils as u


def generate_figure():
    return u.density_contour_figure('100', 'fig06')


if __name__ == '__main__':
    u.run_script(generate_figure)
