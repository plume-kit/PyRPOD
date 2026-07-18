# ========================
# PyRPOD: tests/plume/plume_verification_test_12.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 10: relative density error between the
# analytical and simplified analytical solutions, |n_A/n_As - 1|
# [Eq. 29], Kn = 100 (analytic content is Kn-independent), S0 = 2.0,
# X/D and Y/D in [0, 10].
#
# The paper notes the upper-nozzle-lip region is a density singularity
# whose error pattern "shall be neglected".
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_12.py
# Digitized overlays looked for: fig10_*.csv.

import numpy as np

import plume_figure_utils as u


def generate_figure():
    S_0 = 2.0
    x, y = u.default_contour_axes()
    n_full = u.full_field_grid('n', x, y, S_0)
    n_simp = u.simplified_field_grid('n', x, y, S_0)
    error = np.abs(n_full / n_simp - 1)

    fig, ax = u.plt.subplots(figsize=(6, 5))
    levels = [0.005, 0.03, 0.05, 0.1, 0.2, 0.5, 1, 2]
    cs = ax.contour(x, y, error, levels=levels, colors='k', linewidths=1.0)
    ax.clabel(cs, fmt='%g', fontsize=7)
    u.overlay_digitized(ax, 'fig10', style='line')

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_xlabel('X/D')
    ax.set_ylabel('Y/D')
    ax.set_title('Relative density error: analytical vs simplified '
                 '(Fig. 10)')
    u.annotate_error(
        ax, f'max |n_A/n_As - 1| = {np.nanmax(error):.2g} '
            '(peak at the nozzle-lip singularity)', loc='lower right')
    return u.save_figure(fig, 'fig10_error_analytical_vs_simplified')


if __name__ == '__main__':
    u.run_script(generate_figure)
