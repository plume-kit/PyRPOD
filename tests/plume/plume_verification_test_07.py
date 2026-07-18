# ========================
# PyRPOD: tests/plume/plume_verification_test_07.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 5: normalized analytical temperature
# along the centerline (Eq. 21, exact quadrature), T1/T0 vs X/D for
# S0 = 1, 2, 3.
#
# Manual-run verification script (design decision D5): it defines no
# pytest tests and is executed directly --
#     python tests/plume/plume_verification_test_07.py
# Digitized overlay looked for: fig05_dsmc.csv (S0 = 2 DSMC circles).

import numpy as np

import plume_figure_utils as u


def generate_figure():
    x_over_D = np.linspace(u.X_MIN_OVER_D, 10.0, 200)

    fig, ax = u.plt.subplots(figsize=(6, 4.5))
    for S_0, ls in [(1.0, '-.'), (2.0, '--'), (3.0, '-')]:
        T = u.centerline_profiles(x_over_D, S_0, ('T',))['T']
        ax.plot(x_over_D, T, 'k', ls=ls, lw=1.2, label=f'$S_0$={S_0:g}')
    u.overlay_digitized(ax, 'fig05')

    ax.set_xlim(0, 10)
    ax.set_ylim(0.2, 1.2)
    ax.set_xlabel('X/D')
    ax.set_ylabel('$T_1/T_0$')
    ax.set_title('Normalized analytical temperature along centerline '
                 '(Fig. 5)')
    ax.legend(fontsize=8)
    return u.save_figure(fig, 'fig05_centerline_temperature')


if __name__ == '__main__':
    u.run_script(generate_figure)
