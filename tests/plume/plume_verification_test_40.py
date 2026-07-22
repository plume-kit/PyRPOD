# ========================
# PyRPOD: tests/plume/plume_verification_test_40.py
# ========================
# Reproduces Cai 2016 Fig. 16: static-pressure contours p/p0 in the
# vertical Y = 0 plane for the Section-4 round jet (argon, D = 1 m,
# S0 = 2.0) impinging on the inclined SPECULAR rectangular plate
# (alpha0 = 60 deg, L = 4D) -- the free-jet field plus the paper's 3D
# virtual nozzle at (L(1 - cos 2a0), 0, -L sin 2a0) with mirrored drift
# (see the reference module docstring for the drift-sign resolution),
# whose mirror symmetry about the plate plane the module's sanity
# checks assert to machine precision; peak p/p0 ~ 0.50 vs the paper's
# 0.4 innermost contour. Delegates to
# pyrpod/plume/CaiImpingement2016.flowfield_pressure_plane (single
# source of truth).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_40.py
# Digitized overlays looked for: cai16_fig16_*.csv (one CSV per
# digitized contour polyline).

import plume_impingement_utils as u


def generate_figure():
    return u.pressure_contour_figure_3d(
        'specular', [0.01, 0.05, 0.1, 0.2, 0.3, 0.4], 'cai16_fig16',
        'cai16_fig16_pressure_specular',
        'Round jet on specular plate, $p/p_0$ in $Y$=0 plane, '
        '$S_0$=2.0, $\\alpha_0$=60$^\\circ$ (Fig. 16)')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
