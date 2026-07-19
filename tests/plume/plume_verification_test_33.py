# ========================
# PyRPOD: tests/plume/plume_verification_test_33.py
# ========================
# Reproduces Cai 2016 Fig. 5: temperature contours T/T0 for the 2D slot
# jet (S0 = 2.0) impinging on the inclined DIFFUSE planar plate
# (alpha0 = 60 deg, Tw/T0 = 1.5, L = 4*(2H), W = 5*(2H)) -- the combined
# free-jet + wall-emission field of Section 3, with the stagnation
# heating peaking at T/T0 ~ 2.4 in front of the plate. Delegates to
# pyrpod/plume/CaiImpingement2016.planar_flowfield (single source of
# truth); the analytic field is evaluated on the whole X > 0 plane with
# no shadowing, exactly as the paper's contours are drawn.
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_33.py
# Digitized overlays looked for: cai16_fig05_*.csv (one CSV per
# digitized contour polyline).

import plume_impingement_utils as u


def generate_figure():
    return u.planar_temperature_contour_figure(
        'diffuse', [1.0, 1.4, 1.8, 2.0, 2.2, 2.4], 'cai16_fig05',
        'cai16_fig05_planar_diffuse_temperature',
        '2D jet on diffuse plate, $T/T_0$, $S_0$=2.0, '
        '$\\alpha_0$=60$^\\circ$, $T_w/T_0$=1.5 (Fig. 5)')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
