# ========================
# PyRPOD: tests/plume/plume_verification_test_05.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 3: normalized analytical number
# density along the centerline (Eq. 18) vs X/D for S0 = 1, 2, 3.
#
# Manual-run verification script (design decision D5): it defines no
# pytest tests and is executed directly --
#     python tests/plume/plume_verification_test_05.py
# Digitized overlay looked for: fig03_dsmc.csv (the paper's S0 = 2
# DSMC circles).

import numpy as np

import plume_figure_utils as u


def generate_figure():
    x_over_D = np.linspace(u.X_MIN_OVER_D, 10.0, 200)

    fig, ax = u.plt.subplots(figsize=(6, 4.5))
    for S_0, ls in [(1.0, '-.'), (2.0, '--'), (3.0, '-')]:
        n = u.centerline_profiles(x_over_D, S_0, ('n',))['n']
        ax.plot(x_over_D, n, 'k', ls=ls, lw=1.2, label=f'$S_0$={S_0:g}')
    u.overlay_digitized(ax, 'fig03')

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 1.2)
    ax.set_xlabel('X/D')
    ax.set_ylabel('$n_1/n_0$')
    ax.set_title('Normalized analytical number density along centerline '
                 '(Fig. 3)')
    ax.legend(fontsize=8)
    return u.save_figure(fig, 'fig03_centerline_density')


if __name__ == '__main__':
    u.run_script(generate_figure)
