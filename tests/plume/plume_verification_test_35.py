# ========================
# PyRPOD: tests/plume/plume_verification_test_35.py
# ========================
# Reproduces Cai 2016 Fig. 7: 2D diffuse-plate surface pressure
# profiles Cp,d(s) (Eq. 2, with the A0-integral non-penetration wall
# density) for (S0, alpha0) = (2.0, 60), (2.0, 30), (1.5, 60),
# (1.5, 30) deg at Tw/T0 = 1.5 -- the profiles skew with smaller
# inclination angle, peaking at ~0.93 for the DSMC-validated
# (2.0, 60 deg) case. Delegates to
# pyrpod/plume/CaiImpingement2016.planar_surface_coefficients (single
# source of truth).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_35.py
# Digitized overlays looked for: cai16_fig07_*.csv (e.g.
# cai16_fig07_dsmc.csv, the paper's S0 = 2.0 / 60 deg DSMC symbols).

import plume_impingement_utils as u

PARAMS = [(2.0, 60.0), (2.0, 30.0), (1.5, 60.0), (1.5, 30.0)]


def generate_figure():
    return u.planar_profile_figure(
        'Cp_d', PARAMS, 'cai16_fig07', 'cai16_fig07_planar_cp_diffuse',
        'Diffuse planar plate $C_{p,d}(s)$, $T_w/T_0$=1.5 (Fig. 7)',
        '$C_p$', (0.0, 1.0))


if __name__ == '__main__':
    u.base.run_script(generate_figure)
