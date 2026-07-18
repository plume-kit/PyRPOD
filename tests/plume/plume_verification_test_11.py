# ========================
# PyRPOD: tests/plume/plume_verification_test_11.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 9: relative density error between the
# analytical and Simons (cosine-law) solutions, |n_A/n_Simons - 1|
# [Eq. 28], Kn = 100 (analytic content is Kn-independent), S0 = 2.0,
# X/D and Y/D in [0, 10]. The Simons field is exit-referenced with the
# Boyton kappa (see plume_figure_utils docstring); the paper's A
# normalization additionally depends on exit Mach number, so absolute
# error magnitudes may differ somewhat from the printed figure.
#
# The paper notes the upper-nozzle-lip region is a density singularity
# whose error pattern "shall be neglected".
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_11.py
# Digitized overlays looked for: fig09_*.csv.

import numpy as np

import plume_figure_utils as u


def generate_figure():
    S_0 = 2.0
    x, y = u.default_contour_axes()
    n_full = u.full_field_grid('n', x, y, S_0)
    Xg, Yg = np.meshgrid(x * u.D_NOZZLE, y * u.D_NOZZLE)
    n_simons = u.simons_density_field(Xg, Yg, S_0)
    error = np.abs(n_full / n_simons - 1)

    fig, ax = u.plt.subplots(figsize=(6, 5))
    levels = [0.1, 0.4, 1, 2, 4, 5]
    cs = ax.contour(x, y, error, levels=levels, colors='k', linewidths=1.0)
    ax.clabel(cs, fmt='%g', fontsize=7)
    u.overlay_digitized(ax, 'fig09', style='line')

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_xlabel('X/D')
    ax.set_ylabel('Y/D')
    ax.set_title('Relative density error: analytical vs Simons (Fig. 9)')
    u.annotate_error(
        ax, f'max |n_A/n_Sim - 1| = {np.nanmax(error):.2g} '
            '(peak at the nozzle-lip singularity)', loc='lower right')
    return u.save_figure(fig, 'fig09_error_analytical_vs_simons')


if __name__ == '__main__':
    u.run_script(generate_figure)
