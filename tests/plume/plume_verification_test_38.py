# ========================
# PyRPOD: tests/plume/plume_verification_test_38.py
# ========================
# Reproduces Cai 2016 Fig. 10: 2D diffuse-plate surface heat-flux
# profiles Cq,d(s) (Eq. 4, incoming energy flux minus the wall-Maxwellian
# effusion at Tw) for (S0, alpha0) = (2.0, 60), (2.0, 90), (1.5, 60),
# (1.5, 90) deg at Tw/T0 = 1.5 -- the larger S0, the larger the heat
# flux, peaking at ~0.27 for the DSMC-validated (2.0, 60 deg) case.
# Delegates to
# pyrpod/plume/CaiImpingement2016.planar_surface_coefficients (single
# source of truth).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_38.py
# Digitized overlays looked for: cai16_fig10_*.csv (e.g.
# cai16_fig10_dsmc.csv).

import plume_impingement_utils as u

PARAMS = [(2.0, 60.0), (2.0, 90.0), (1.5, 60.0), (1.5, 90.0)]


def generate_figure():
    return u.planar_profile_figure(
        'Cq_d', PARAMS, 'cai16_fig10', 'cai16_fig10_planar_cq_diffuse',
        'Diffuse planar plate $C_{q,d}(s)$, $T_w/T_0$=1.5 (Fig. 10)',
        '$C_q$', (0.0, 0.3))


if __name__ == '__main__':
    u.base.run_script(generate_figure)
