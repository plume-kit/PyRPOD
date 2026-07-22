# ========================
# PyRPOD: tests/plume/plume_verification_test_34.py
# ========================
# Reproduces Cai 2016 Fig. 6: temperature contours T/T0 for the 2D slot
# jet (S0 = 2.0) impinging on the inclined SPECULAR planar plate
# (alpha0 = 60 deg, L = 4*(2H)) -- the free jet plus the paper's
# virtual-nozzle construction (mirrored exit and drift, Eq. 7), whose
# defining property is the contour symmetry about the plate line that
# the module's sanity checks assert to machine precision; counterflow
# stagnation heating peaks at T/T0 ~ 3.5. Delegates to
# pyrpod/plume/CaiImpingement2016.planar_flowfield (single source of
# truth).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_34.py
# Digitized overlays looked for: cai16_fig06_*.csv (one CSV per
# digitized contour polyline).

import plume_impingement_utils as u


def generate_figure():
    return u.planar_temperature_contour_figure(
        'specular', [1.0, 1.5, 2.0, 2.5, 3.0, 3.5], 'cai16_fig06',
        'cai16_fig06_planar_specular_temperature',
        '2D jet on specular plate, $T/T_0$, $S_0$=2.0, '
        '$\\alpha_0$=60$^\\circ$ (Fig. 6)')


if __name__ == '__main__':
    u.base.run_script(generate_figure)
