# ========================
# PyRPOD: tests/plume/plume_verification_test_26.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 24: density profiles along r/D = 10,
# Kn = 0.01, S0 = 2.0. Same analytic content as Fig. 22; the Kn
# distinction lives in the DSMC overlay slot.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_26.py
# Digitized overlay looked for: fig24_dsmc.csv.

import plume_figure_utils as u


def generate_figure():
    return u.angular_density_profile_figure('0.01', 'fig24')


if __name__ == '__main__':
    u.run_script(generate_figure)
