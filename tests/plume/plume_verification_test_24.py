# ========================
# PyRPOD: tests/plume/plume_verification_test_24.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 22: density profiles along r/D = 10,
# Kn = 100, S0 = 2.0 -- analytical, simplified, Simons kappa = 1.5/2/3
# and the DSMC digitized slot. Following the paper (p. 65), the Simons
# curves share one normalization constant A (Boyton kappa) so they
# coincide at theta = 0; only the plotted decay exponent varies.
# Analytic content is Kn-independent; Figs. 22-24 differ only in their
# DSMC data.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_24.py
# Digitized overlay looked for: fig22_dsmc.csv.

import plume_figure_utils as u


def generate_figure():
    return u.angular_density_profile_figure('100', 'fig22')


if __name__ == '__main__':
    u.run_script(generate_figure)
