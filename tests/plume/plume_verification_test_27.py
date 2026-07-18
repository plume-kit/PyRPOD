# ========================
# PyRPOD: tests/plume/plume_verification_test_27.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 25: normalized mass flux along
# r/D = 10 vs theta, S0 = 2.0 -- the analytical curve plus three DSMC
# digitized slots (Kn = 100, 0.1, 0.01).
#
# Normalization note: the paper states the flux is "normalized by the
# inflow mass flux at the exit plane" (p. 66), which would suggest
# rho*Vr / (rho0*U0) = (n/n0)*(Vr*sqrt(beta0))/S0. That convention
# peaks at ~0.0137 at theta = 0, half the paper's ~0.027. The plotted
# magnitudes of Fig. 25 are instead reproduced exactly by
# (n/n0)*(Vr*sqrt(beta0)), i.e. rho*Vr normalized by
# rho0*sqrt(2*R*T0) = rho0*U0/S0. That convention is adopted here
# (verified: theta = 0 value 0.0273 vs the paper's ~0.027); it should
# be re-calibrated against the digitized DSMC data when available.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_27.py
# Digitized overlays looked for: fig25_dsmc_kn100.csv,
# fig25_dsmc_kn0p1.csv, fig25_dsmc_kn0p01.csv.

import numpy as np

import plume_figure_utils as u


def mass_flux_curve(theta_deg, S_0=2.0, r_over_D=10.0):
    """(n/n0)*(Vr*sqrt(beta0)) along the angular curve (see header)."""
    theta = np.deg2rad(theta_deg)
    r = r_over_D * u.D_NOZZLE
    X = r * np.cos(theta)
    Z = r * np.sin(theta)
    n = u.full_field_values('n', X, Z, S_0)
    Vr = u.full_field_values('Vr', X, Z, S_0)
    return n * Vr


def generate_figure():
    theta_deg = np.linspace(0.0, u.THETA_MAX_DEG, 90)
    flux = mass_flux_curve(theta_deg)

    fig, ax = u.plt.subplots(figsize=(6, 4.5))
    ax.plot(theta_deg, flux, linestyle='none', marker='o', mfc='none',
            color='k', ms=4, markevery=2, label='Analytical')
    u.overlay_digitized(ax, 'fig25', style='line')

    ax.set_xlim(0, 90)
    ax.set_ylim(0, 0.03)
    ax.set_xticks([0, 30, 60, 90])
    ax.set_xlabel(r'$\theta$')
    ax.set_ylabel('Mass flux')
    ax.set_title('Normalized mass flux along r/D = 10 (Fig. 25)')
    ax.legend(fontsize=8)
    return u.save_figure(fig, 'fig25_mass_flux')


if __name__ == '__main__':
    u.run_script(generate_figure)
