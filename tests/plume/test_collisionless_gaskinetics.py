# ========================
# PyRPOD: tests/plume/test_collisionless_gaskinetics.py
# ========================
# Verification tests for the full collisionless analytical plume model
# CollisionlessGasKinetics (Cai & Wang 2012, Eqs. 5-12), checked against
# the paper's own internal anchors:
#   (a, b) the exact field integrals must reduce to the Eq. 18/19
#          closed forms on the centerline,
#   (c)    W must vanish on the centerline (Eq. 20),
#   (d)    the full and simplified models must agree in the far field
#          within a bound derived from Eq. 30,
#   (e)    qualitative physics of Figs. 3, 5, 13 (monotonic centerline
#          density decay, cooling below T_0),
#   (f)    convergence to the Eq. 22-24 far-field asymptotes.

import numpy as np
import pytest

from pyrpod.plume.RarefiedPlumeGasKinetics import (
    CollisionlessGasKinetics,
    SimplifiedGasKinetics,
    get_far_field_temp_ratio,
    get_far_field_velocity_normalized,
)

pytestmark = pytest.mark.plume

R_SPECIFIC = 208.13   # J / (kg K), argon-like
T_0 = 500.0           # K
GAMMA = 5.0 / 3.0
N_0 = 1.0e20          # m^-3
D_NOZZLE = 0.2        # m  (R_0 = 0.1 m)
R_0 = D_NOZZLE / 2
T_W = 300.0           # K
SIGMA = 1.0

# The class doubles the quadrature order until successive orders agree
# to QUAD_RTOL = 1e-9; comparisons against exact closed forms therefore
# use a slightly looser 1e-8.
RTOL_QUAD = 1e-8


def make_plume(cls, distance, theta, S_0):
    ve = S_0 * np.sqrt(2 * R_SPECIFIC * T_0)
    thruster_characteristics = {
        'd': D_NOZZLE, 've': ve, 'R': R_SPECIFIC,
        'gamma': GAMMA, 'Te': T_0, 'n': N_0,
    }
    return cls(distance, theta, thruster_characteristics, T_W, SIGMA)


CENTERLINE_POINTS = [(0.2, 1.0), (0.5, 2.0), (1.0, 2.0), (5.0, 2.0),
                     (1.0, 3.0)]


@pytest.mark.parametrize("X,S_0", CENTERLINE_POINTS)
def test_centerline_density_matches_eq18(X, S_0):
    """(a) The Eq. 5 double integral evaluated at (X, 0, 0) must equal
    the Eq. 18 closed form (they were derived from the same solution)."""
    plume = make_plume(CollisionlessGasKinetics, X, 0.0, S_0)
    assert plume.get_num_density_ratio() == pytest.approx(
        plume.get_num_density_centerline(), rel=RTOL_QUAD)


@pytest.mark.parametrize("X,S_0", CENTERLINE_POINTS)
def test_centerline_velocity_matches_eq19(X, S_0):
    """(b) The Eq. 6 integral at (X, 0, 0) must equal the Eq. 19 closed
    form."""
    plume = make_plume(CollisionlessGasKinetics, X, 0.0, S_0)
    assert plume.get_U_normalized() == pytest.approx(
        plume.get_velocity_centerline(), rel=RTOL_QUAD)


@pytest.mark.parametrize("X,S_0", CENTERLINE_POINTS)
def test_centerline_W_vanishes(X, S_0):
    """(c) Eq. 20: W(X, 0, 0) = 0. The quadrature can only leave
    rounding noise, so |W| is required to be negligible against U. On
    the centerline V_r must also reduce to U."""
    plume = make_plume(CollisionlessGasKinetics, X, 0.0, S_0)
    U = plume.get_U_normalized()
    assert abs(plume.get_W_normalized()) < 1e-12 * U
    assert plume.get_Vr_normalized() == pytest.approx(U, rel=1e-12)


# Far-field points (distance, theta) with S_0 = 2.
FAR_FIELD_POINTS = [(20.0, 0.4), (50.0, 0.8), (100.0, 0.2)]


@pytest.mark.parametrize("distance,theta", FAR_FIELD_POINTS)
def test_far_field_agrees_with_simplified_within_eq30_bound(distance, theta):
    """(d) Eq. 30 bounds the pointwise error of the simplified Q':
    |Q' - Q|/Q <= max(R_0^2, 2*Z*R_0)/(X^2 + Z^2) over the exit disk.
    The K, M, N factors depend on Q through powers up to Q^(5/2), the
    factor exp(S_0^2 * Q), and bounded erf terms, so their logarithmic
    sensitivity |d ln K / d ln Q| is at most ~(S_0^2 + 4). The relative
    difference between the full and simplified field quantities is
    therefore bounded by (S_0^2 + 4) * max(R_0^2, 2*Z*R_0)/(X^2 + Z^2)."""
    S_0 = 2.0
    full = make_plume(CollisionlessGasKinetics, distance, theta, S_0)
    simple = make_plume(SimplifiedGasKinetics, distance, theta, S_0)

    bound = ((S_0 ** 2 + 4)
             * max(R_0 ** 2, 2 * full.Z * R_0) / (full.X ** 2 + full.Z ** 2))

    assert full.get_num_density_ratio() == pytest.approx(
        simple.get_num_density_ratio(), rel=bound)
    assert full.get_U_normalized() == pytest.approx(
        simple.get_U_normalized(), rel=bound)
    assert full.get_W_normalized() == pytest.approx(
        simple.get_W_normalized(), rel=bound)
    assert full.get_temp_ratio() == pytest.approx(
        simple.get_temp_ratio(), rel=bound)


def test_centerline_density_decays_monotonically():
    """(e) Paper Fig. 3: for fixed S_0 the centerline density decreases
    monotonically with X."""
    S_0 = 2.0
    X_values = [0.15, 0.3, 0.6, 1.2, 2.5, 5.0, 10.0, 20.0]
    densities = [make_plume(CollisionlessGasKinetics, X, 0.0, S_0)
                 .get_num_density_ratio() for X in X_values]
    assert all(a > b for a, b in zip(densities, densities[1:]))
    assert all(n > 0 for n in densities)


@pytest.mark.parametrize("distance,theta", [(0.2, 0.0), (1.0, 0.0),
                                            (1.0, 0.5), (5.0, 1.0)])
def test_temperature_below_exit_temperature_downstream(distance, theta):
    """(e) Paper Figs. 5, 13: the collisionless expansion only cools the
    gas, so T_1 < T_0 everywhere downstream; consequently the static
    pressure ratio p_1/p_0 = (n/n_0)(T/T_0) lies below the density
    ratio (the paper's p. 64 argument against using n*k*T_0)."""
    plume = make_plume(CollisionlessGasKinetics, distance, theta, 2.0)
    T_ratio = plume.get_temp_ratio()
    assert 0.0 < T_ratio < 1.0
    assert 0.0 < plume.get_pressure_ratio() < plume.get_num_density_ratio()


@pytest.mark.parametrize("S_0", [1.0, 2.0, 3.0])
def test_centerline_converges_to_far_field_asymptotes(S_0):
    """(f) Full-model centerline U and T converge to the Eq. 22-24
    asymptotes at large X. Tolerances follow the O((R_0/X)^2)
    convergence-rate arguments in test_simplified_gaskinetics.py
    (rel=1e-5 for U at X/R_0 = 1000; rel=1e-4 for T at X/R_0 = 5000
    due to the -2/3 U^2 cancellation amplification)."""
    plume_U = make_plume(CollisionlessGasKinetics, 1000 * R_0, 0.0, S_0)
    assert plume_U.get_U_normalized() == pytest.approx(
        get_far_field_velocity_normalized(S_0), rel=1e-5)

    plume_T = make_plume(CollisionlessGasKinetics, 5000 * R_0, 0.0, S_0)
    assert plume_T.get_temp_ratio() == pytest.approx(
        get_far_field_temp_ratio(S_0), rel=1e-4)


def test_surface_interaction_uses_full_model_and_is_finite():
    """The inherited Maxwell gas-surface interface dispatches to the
    overridden full-model field getters; off the centerline it must
    return finite, physically-signed values (p > 0, q finite, shear
    opposing the flow direction)."""
    plume = make_plume(CollisionlessGasKinetics, 1.0, 0.4, 2.0)
    p = float(plume.get_pressure())
    tau = float(plume.get_shear_pressure())
    q = float(plume.get_heat_flux())
    assert np.isfinite(p) and p > 0.0
    assert np.isfinite(tau) and tau < 0.0
    assert np.isfinite(q)
