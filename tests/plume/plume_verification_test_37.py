# ========================
# PyRPOD: tests/plume/plume_verification_test_37.py
# ========================
# Reproduces Cai 2016 Fig. 9: 2D diffuse-plate surface friction
# profiles Cf,d(s) (Eq. 3) for (S0, alpha0) = (2.0, 60), (2.0, 30),
# (1.5, 60), (1.5, 30) deg at Tw/T0 = 1.5. Smaller inclination angles
# create larger friction; the (2.0, 60 deg) profile crosses zero near
# s/(2H) ~ -2 (the paper's possible flow-separation spot; the module
# anchor reproduces it at -2.12). Delegates to
# pyrpod/plume/CaiImpingement2016.planar_surface_coefficients (single
# source of truth).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_37.py
# Digitized overlays looked for: cai16_fig09_*.csv (e.g.
# cai16_fig09_dsmc.csv).

import plume_impingement_utils as u

PARAMS = [(2.0, 60.0), (2.0, 30.0), (1.5, 60.0), (1.5, 30.0)]


def generate_figure():
    return u.planar_profile_figure(
        'Cf_d', PARAMS, 'cai16_fig09', 'cai16_fig09_planar_cf_diffuse',
        'Diffuse planar plate $C_{f,d}(s)$, $T_w/T_0$=1.5 (Fig. 9)',
        '$C_f$', (-0.1, 0.4))


if __name__ == '__main__':
    u.base.run_script(generate_figure)
