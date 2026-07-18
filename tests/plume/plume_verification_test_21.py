# ========================
# PyRPOD: tests/plume/plume_verification_test_21.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 19: centerline density profiles,
# Kn = 100, S0 = 2.0 -- analytical (Eq. 18), simplified (Eq. 14),
# Simons (exit-referenced cosine law) and the DSMC digitized slot.
# Analytic content is Kn-independent; Figs. 19-21 differ only in their
# DSMC data.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_21.py
# Digitized overlay looked for: fig19_dsmc.csv.

import plume_figure_utils as u


def generate_figure():
    return u.centerline_density_profile_figure('100', 'fig19')


if __name__ == '__main__':
    u.run_script(generate_figure)
