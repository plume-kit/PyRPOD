# ========================
# PyRPOD: tests/plume/test_simplified_gaskinetics.py
# ========================
# Pinning tests for the SimplifiedGasKinetics class (Cai & Wang 2012,
# "Numerical Validations for a Set of Collisionless Rocket Plume Solutions",
# JSR 49(1), DOI 10.2514/1.A32046).
#
# Reference values were computed independently from the paper's equations
# with mpmath at 40 significant digits (Eqs. 13-19 and 22-24 implemented
# directly from the printed formulas, not from pyrpod code). They pin the
# known-correct behavior of the simplified far-field model so that later
# refactors (sympy removal, overflow-safe exponentials) can be verified
# to preserve behavior.

import numpy as np
import pytest

from pyrpod.plume.RarefiedPlumeGasKinetics import (
    SimplifiedGasKinetics,
    get_far_field_temp_ratio,
    get_far_field_velocity_normalized,
)

pytestmark = pytest.mark.plume

# Fixed thruster gas properties used for all cases (argon-like).
R_SPECIFIC = 208.13   # J / (kg K)
T_0 = 500.0           # K
GAMMA = 5.0 / 3.0
N_0 = 1.0e20          # m^-3
D_NOZZLE = 0.2        # m  (R_0 = 0.1 m)
T_W = 300.0           # K
SIGMA = 1.0

# Relative tolerance for pinned closed-form values: the implementation
# evaluates the same closed-form expressions in float64 (machine epsilon
# ~1e-16, tens of flops => accumulated rounding well below 1e-12), while
# the references carry 17 correct digits. 1e-9 leaves ample headroom for
# rounding-level refactors (e.g. sympy.erf -> scipy.special.erf, combined
# exponentials) yet fails loudly on any actual formula change.
RTOL_PINNED = 1e-9


def make_plume(distance, theta, S_0):
    """Build a SimplifiedGasKinetics instance with exit speed ratio S_0."""
    ve = S_0 * np.sqrt(2 * R_SPECIFIC * T_0)
    thruster_characteristics = {
        'd': D_NOZZLE, 've': ve, 'R': R_SPECIFIC,
        'gamma': GAMMA, 'Te': T_0, 'n': N_0,
    }
    return SimplifiedGasKinetics(distance, theta, thruster_characteristics,
                                 T_W, SIGMA)


# (distance, theta, S_0) -> (n/n0, U*sqrt(beta0), W*sqrt(beta0), T/T0)
# per Eqs. 14-17 with Q' = X^2/(X^2+Z^2) (Eq. 13).
FIELD_CASES = [
    ((1.0, 0.2, 2.0), (0.036338635060152093, 2.3636427215997389,
                       0.47913410002529834, 0.27393936134733628)),
    ((2.0, 0.7, 2.0), (0.0010317934247266036, 1.5832262838600719,
                       1.3335331025390796, 0.2543577108315116)),
    ((5.0, 1.2, 2.0), (4.4418364832646119e-6, 0.54854957955069592,
                       1.4109526908580327, 0.20360334802441553)),
    ((1.5, 0.4, 1.0), (0.0046772248683472122, 1.5085528987977848,
                       0.63780593571949983, 0.21734975912503903)),
    ((3.0, 0.9, 3.0), (1.0977476844642729e-5, 1.4507999313815155,
                       1.8282374555518849, 0.27008477291007084)),
]

# (X, S_0) -> (n/n0, U*sqrt(beta0)) per Eqs. 18-19 (centerline).
CENTERLINE_CASES = [
    ((0.5, 2.0), (0.15923868831612018, 2.4062627987321269)),
    ((1.0, 2.0), (0.043598168447392599, 2.434502010083779)),
    ((5.0, 2.0), (0.0017976259018792059, 2.4441625633761652)),
    ((1.0, 1.0), (0.014624592798873514, 1.6841200399767633)),
    ((1.0, 3.0), (0.089793762869630585, 3.3010882334220839)),
]

# S_0 -> (U_inf*sqrt(beta0), T_inf/T0) per Eqs. 22-24, evaluated with
# mpmath. (Eq. 23's denominator is 3*[S0 + (1/2+S0^2)*sqrt(pi)*
# (1+erf(S0))*exp(S0^2)], i.e. 3*K(Q=1), consistent with
# -2/3*G^2 + 4N(1)/(3K(1)).)
ASYMPTOTE_CASES = {
    1.0: (1.689948557881606, 0.22268161973032418),
    2.0: (2.444572023854802, 0.2754744452640276),
    3.0: (3.3157896665554836, 0.30193859122021722),
}


@pytest.mark.parametrize("point,expected", FIELD_CASES)
def test_field_solution_pinned(point, expected):
    """Pin Eqs. 14-17 (off-centerline simplified field solutions)."""
    distance, theta, S_0 = point
    n_ref, U_ref, W_ref, T_ref = expected
    plume = make_plume(distance, theta, S_0)
    assert plume.get_num_density_ratio() == pytest.approx(n_ref, rel=RTOL_PINNED)
    assert plume.get_U_normalized() == pytest.approx(U_ref, rel=RTOL_PINNED)
    assert plume.get_W_normalized() == pytest.approx(W_ref, rel=RTOL_PINNED)
    assert plume.get_temp_ratio() == pytest.approx(T_ref, rel=RTOL_PINNED)


@pytest.mark.parametrize("point,expected", CENTERLINE_CASES)
def test_centerline_solution_pinned(point, expected):
    """Pin Eqs. 18-19 (centerline density and velocity closed forms)."""
    X, S_0 = point
    n_ref, U_ref = expected
    plume = make_plume(X, 0.0, S_0)
    assert plume.get_num_density_centerline() == pytest.approx(n_ref,
                                                               rel=RTOL_PINNED)
    assert plume.get_velocity_centerline() == pytest.approx(U_ref,
                                                            rel=RTOL_PINNED)


@pytest.mark.parametrize("S_0", [1.0, 2.0, 3.0])
def test_far_field_asymptote_functions_pinned(S_0):
    """Pin the Eq. 22-24 module-level asymptote functions against the
    independent mpmath evaluation (tolerance rationale as RTOL_PINNED)."""
    U_ref, T_ref = ASYMPTOTE_CASES[S_0]
    assert get_far_field_velocity_normalized(S_0) == pytest.approx(
        U_ref, rel=RTOL_PINNED)
    assert get_far_field_temp_ratio(S_0) == pytest.approx(
        T_ref, rel=RTOL_PINNED)


@pytest.mark.parametrize("S_0", [1.0, 2.0, 3.0])
def test_centerline_velocity_approaches_asymptote(S_0):
    """Centerline U converges to the Eq. 22/24 far-field asymptote.

    Convergence is O((R_0/X)^2): at X/R_0 = 1000 the residual is ~4e-7
    relative, so rel=1e-5 passes with an order-of-magnitude margin while
    still requiring genuine convergence.
    """
    U_inf = get_far_field_velocity_normalized(S_0)
    X = 1000 * (D_NOZZLE / 2)
    plume = make_plume(X, 0.0, S_0)
    assert plume.get_velocity_centerline() == pytest.approx(U_inf, rel=1e-5)


@pytest.mark.parametrize("S_0", [1.0, 2.0, 3.0])
def test_centerline_temperature_approaches_asymptote(S_0):
    """Centerline T converges to the Eq. 23 far-field asymptote.

    T = -2/3*U^2 + 4/3*<N>/<K> subtracts two O(S_0^2) quantities, so the
    O((R_0/X)^2 * S_0^2) residual of each term is amplified by ~U^2/T
    (a factor ~25 at S_0 = 3) in the relative error of T. At
    X/R_0 = 5000 the amplified residual is <~1e-5 for S_0 <= 3, so
    rel=1e-4 passes with margin for both the constant-N approximation
    and the exact Eq. 21 quadrature, while still requiring convergence.
    """
    T_inf = get_far_field_temp_ratio(S_0)
    X = 5000 * (D_NOZZLE / 2)
    plume = make_plume(X, 0.0, S_0)
    assert plume.get_temp_centerline() == pytest.approx(T_inf, rel=1e-4)


@pytest.mark.parametrize("point,expected", [
    ((1.0, 0.2, 2.0), (0.36855461276327334, -0.056815859301478394,
                       143.3923778236429)),
    ((2.0, 0.7, 2.0), (0.005299929030826453, -0.003007472717182616,
                       1.9141355894449432)),
])
def test_surface_interaction_pinned(point, expected):
    """Pin the Maxwell gas-surface interface outputs (off-centerline).

    Unlike the field-solution pins above, these reference values were
    generated from the current implementation (not independently), so
    they pin implementation behavior for refactor safety rather than
    correctness against the paper. Tolerance rationale as RTOL_PINNED.
    """
    distance, theta, S_0 = point
    p_ref, tau_ref, q_ref = expected
    plume = make_plume(distance, theta, S_0)
    assert float(plume.get_pressure()) == pytest.approx(p_ref, rel=RTOL_PINNED)
    assert float(plume.get_shear_pressure()) == pytest.approx(tau_ref,
                                                              rel=RTOL_PINNED)
    assert float(plume.get_heat_flux()) == pytest.approx(q_ref, rel=RTOL_PINNED)
