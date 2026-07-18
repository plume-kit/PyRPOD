# ========================
# PyRPOD: tests/plume/plume_verification_test_06.py
# ========================
# Reproduces Cai & Wang 2012 Fig. 4: normalized analytical U-velocity
# along the centerline (Eq. 19), U1*sqrt(beta0) vs X/D, S0 = 1, 2, 3.
#
# Manual-run verification script (design decision D5): it defines no
# pytest tests and is executed directly --
#     python tests/plume/plume_verification_test_06.py
# Digitized overlay looked for: fig04_dsmc.csv (S0 = 2 DSMC circles).

import numpy as np

import plume_figure_utils as u


def generate_figure():
    x_over_D = np.linspace(u.X_MIN_OVER_D, 10.0, 200)

    fig, ax = u.plt.subplots(figsize=(6, 4.5))
    for S_0, ls in [(1.0, '-.'), (2.0, '--'), (3.0, '-')]:
        U = u.centerline_profiles(x_over_D, S_0, ('U',))['U']
        ax.plot(x_over_D, U, 'k', ls=ls, lw=1.2, label=f'$S_0$={S_0:g}')
    u.overlay_digitized(ax, 'fig04')

    ax.set_xlim(0, 10)
    ax.set_ylim(1, 4)
    ax.set_xlabel('X/D')
    ax.set_ylabel(r'$U_1\sqrt{\beta_0}$')
    ax.set_title('Normalized analytical U-velocity along centerline '
                 '(Fig. 4)')
    ax.legend(fontsize=8, loc='lower right')
    return u.save_figure(fig, 'fig04_centerline_velocity')


if __name__ == '__main__':
    u.run_script(generate_figure)
