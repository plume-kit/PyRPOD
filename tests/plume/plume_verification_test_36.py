# ========================
# PyRPOD: tests/plume/plume_verification_test_36.py
# ========================
# Reproduces Cai 2016 Fig. 8: 2D specular-plate surface pressure
# profiles Cp,s(s) (Eq. 8 -- twice the jet-only normal momentum flux,
# wall temperature drops out) for (S0, alpha0) = (2.0, 60), (2.0, 30),
# (1.5, 60), (1.5, 30) deg, peaking at ~1.23 for the DSMC-validated
# (2.0, 60 deg) case. Delegates to
# pyrpod/plume/CaiImpingement2016.planar_surface_coefficients (single
# source of truth).
#
# Manual-run verification script (design decision D5): no pytest tests;
# run directly -- python tests/plume/plume_verification_test_36.py
# Digitized overlays looked for: cai16_fig08_*.csv (e.g.
# cai16_fig08_dsmc.csv).

import plume_impingement_utils as u

PARAMS = [(2.0, 60.0), (2.0, 30.0), (1.5, 60.0), (1.5, 30.0)]


def generate_figure():
    return u.planar_profile_figure(
        'Cp_s', PARAMS, 'cai16_fig08', 'cai16_fig08_planar_cp_specular',
        'Specular planar plate $C_{p,s}(s)$ (Fig. 8)',
        '$C_p$', (0.0, 1.25))


if __name__ == '__main__':
    u.base.run_script(generate_figure)
