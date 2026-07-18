# ========================
# PyRPOD: tests/plume/plume_verification_test_20.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 18: normalized radial velocity Vr
# contours, Kn = 0.01, S0 = 2.0. Same analytic content as Fig. 16; the
# Kn distinction lives in the DSMC overlay slot.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_20.py
# Digitized overlays looked for: fig18_*.csv.

import plume_figure_utils as u

from plume_verification_test_18 import VR_LEVELS


def generate_figure():
    return u.top_bottom_contour_figure(
        'Vr', VR_LEVELS, '0.01', 'fig18',
        'Normalized velocity $V_r$ distribution')


if __name__ == '__main__':
    u.run_script(generate_figure)
