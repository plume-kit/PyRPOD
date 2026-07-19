# ========================
# PyRPOD: tests/plume/plume_verification_test_31.py
# ========================
# Reproduces Cai 2016 Fig. 20: diffuse-plate friction coefficient
# Cf2,d(s, tau) along the horizontal direction (Eq. 12, solid black) at
# the Section-4 conditions -- antisymmetric in s (gas flows outward to
# both sides of the vertical symmetry plane). Overlaid dashed red: the
# current PyRPOD approximation, decomposed onto the horizontal plate
# axis (see plume_verification_test_30 header).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_31.py
# Digitized overlays looked for: fig20_*.csv (one CSV per digitized
# contour polyline).

import plume_impingement_utils as u


def generate_figure():
    ref = u.reference_grid()
    chain = u.chain_grid()
    levels = [-0.02, -0.01, -0.005, -0.001,
              0.001, 0.005, 0.01, 0.02]
    return u.impingement_contour_figure(
        'fig20', ref['Cf2_d'], chain['Cf2'], levels,
        'Diffuse plate $C_{f_2,d}(s,\\tau)$, $S_0$=2.0, '
        '$T_w/T_0$=1.5, $\\alpha_0$=60$^\\circ$ (Fig. 20)',
        'fig20_diffuse_friction_s')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
