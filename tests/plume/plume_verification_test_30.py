# ========================
# PyRPOD: tests/plume/plume_verification_test_30.py
# ========================
# Reproduces Cai 2016 Fig. 19: diffuse-plate friction coefficient
# Cf1,d(s, tau) along the inclined direction (Eq. 11, solid black) at
# the Section-4 conditions. The zero contour is the stagnation line
# (tau ~ -2): gas flows up-plate above it (positive Cf1) and down-plate
# below. Overlaid dashed red: the current PyRPOD approximation -- the
# Shen/Maxwellian shear magnitude at the true incidence angle,
# decomposed onto the inclined plate axis along the tangential
# projection of the radial flow direction.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_30.py
# Digitized overlays looked for: cai16_fig19_*.csv (one CSV per
# digitized contour polyline; cai16_ prefix avoids the 2012 fig19 slot).

import plume_impingement_utils as u


def generate_figure():
    ref = u.reference_grid()
    chain = u.chain_grid()
    levels = [-0.001, 0.0, 0.001, 0.01, 0.05]
    return u.impingement_contour_figure(
        'cai16_fig19', ref['Cf1_d'], chain['Cf1'], levels,
        'Diffuse plate $C_{f_1,d}(s,\\tau)$, $S_0$=2.0, '
        '$T_w/T_0$=1.5, $\\alpha_0$=60$^\\circ$ (Fig. 19)',
        'fig19_diffuse_friction_tau')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
