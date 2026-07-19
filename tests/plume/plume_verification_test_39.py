# ========================
# PyRPOD: tests/plume/plume_verification_test_39.py
# ========================
# Reproduces Cai 2016 Fig. 15: static-pressure contours p/p0 in the
# vertical Y = 0 plane for the Section-4 round jet (argon, D = 1 m,
# S0 = 2.0) impinging on the inclined DIFFUSE rectangular plate
# (alpha0 = 60 deg, L = 4D, Tw/T0 = 1.5) -- the free-jet field (2012
# exit-disk integrals, imported from RarefiedPlumeGasKinetics) combined
# with the Eq.-9 wall-emission population, showing the paper's gas
# accumulation in the impingement center region (peak p/p0 ~ 0.58 just
# below the plate center vs the paper's 0.5 innermost contour).
# Delegates to
# pyrpod/plume/CaiImpingement2016.flowfield_pressure_plane (single
# source of truth).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_39.py
# Digitized overlays looked for: cai16_fig15_*.csv (one CSV per
# digitized contour polyline).

import plume_impingement_utils as u


def generate_figure():
    return u.pressure_contour_figure_3d(
        'diffuse', [0.05, 0.1, 0.2, 0.4, 0.5], 'cai16_fig15',
        'cai16_fig15_pressure_diffuse',
        'Round jet on diffuse plate, $p/p_0$ in $Y$=0 plane, '
        '$S_0$=2.0, $\\alpha_0$=60$^\\circ$ (Fig. 15)')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
