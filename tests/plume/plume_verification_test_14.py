# ========================
# PyRPOD: tests/plume/plume_verification_test_14.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 12: normalized pressure contours
# p1/p0, Kn = 0.1, S0 = 2.0. Same analytic content as Fig. 11; the Kn
# distinction lives in the DSMC overlay slot.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_14.py
# Digitized overlays looked for: fig12_*.csv.

import plume_figure_utils as u


def generate_figure():
    return u.top_bottom_contour_figure(
        'p', [1e-5, 1e-4, 1e-3, 1e-2], '0.1', 'fig12',
        'Normalized pressure contours')


if __name__ == '__main__':
    u.run_script(generate_figure)
