# ========================
# PyRPOD: tests/plume/plume_verification_test_04.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 2: plume boundaries (n/n0 = 0.001
# contour of the full analytical model) for exit speed ratios
# S0 = 1, 2, 3; X/D in [0, 30], Z/D in [0, 15].
#
# Manual-run verification script (design decision D5): it defines no
# pytest tests and is executed directly --
#     python tests/plume/plume_verification_test_04.py
# The figure is saved to tests/plume/output/.
# Digitized overlays looked for: fig02_*.csv (none published -- Fig. 2
# has no DSMC data in the paper).

import numpy as np

import plume_figure_utils as u


def generate_figure():
    x = np.linspace(u.X_MIN_OVER_D, 30.0, 181)
    z = np.linspace(0.0, 15.0, 91)

    fig, ax = u.plt.subplots(figsize=(7, 4.5))
    for S_0, ls in [(1.0, '-'), (2.0, '--'), (3.0, '-.')]:
        n = u.full_field_grid('n', x, z, S_0)
        cs = ax.contour(x, z, n, levels=[0.001], colors='k',
                        linestyles=ls, linewidths=1.2)
        ax.clabel(cs, fmt='0.001', fontsize=7)
        ax.plot([], [], 'k', ls=ls, label=f'$S_0$={S_0:g}')
    u.overlay_digitized(ax, 'fig02')

    ax.set_xlim(0, 30)
    ax.set_ylim(0, 15)
    ax.set_xlabel('X/D')
    ax.set_ylabel('Z/D')
    ax.set_title('Plume boundaries at different exit speed ratios '
                 '(Fig. 2)')
    ax.legend(fontsize=8)
    return u.save_figure(fig, 'fig02_plume_boundaries')


if __name__ == '__main__':
    u.run_script(generate_figure)
