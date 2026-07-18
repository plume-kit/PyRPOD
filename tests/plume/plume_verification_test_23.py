# ========================
# PyRPOD: tests/plume/plume_verification_test_23.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 21: centerline density profiles,
# Kn = 0.01, S0 = 2.0. Same analytic content as Fig. 19; the Kn
# distinction lives in the DSMC overlay slot.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_23.py
# Digitized overlay looked for: fig21_dsmc.csv.

import plume_figure_utils as u


def generate_figure():
    return u.centerline_density_profile_figure('0.01', 'fig21')


if __name__ == '__main__':
    u.run_script(generate_figure)
