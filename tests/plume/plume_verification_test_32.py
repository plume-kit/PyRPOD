# ========================
# PyRPOD: tests/plume/plume_verification_test_32.py
# ========================
# Reproduces Cai 2016 Fig. 21: diffuse-plate heat-flux coefficient
# Cq,d(s, tau) (Eq. 13, solid black) at the Section-4 conditions --
# peak just beneath the plate center, as the paper notes. Overlaid
# dashed red: the current PyRPOD approximation (SimplifiedGasKinetics
# field + Shen/Maxwellian heat transfer at the true incidence angle).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_32.py
# Digitized overlays looked for: fig21_*.csv (one CSV per digitized
# contour polyline).

import plume_impingement_utils as u


def generate_figure():
    ref = u.reference_grid()
    chain = u.chain_grid()
    levels = [0.001, 0.01, 0.05]
    return u.impingement_contour_figure(
        'fig21', ref['Cq_d'], chain['Cq'], levels,
        'Diffuse plate $C_{q,d}(s,\\tau)$, $S_0$=2.0, '
        '$T_w/T_0$=1.5, $\\alpha_0$=60$^\\circ$ (Fig. 21)',
        'fig21_diffuse_heat_flux')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
