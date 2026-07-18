# ========================
# PyRPOD: tests/plume/plume_unit_test_03.py
# ========================
# Tests for the Simons cosine-law plume model (Cai & Wang 2012, Sec. II.C,
# Eqs. 25-26) after the gamma-generalization fixes:
#   - isentropic throat ratios computed from gamma instead of the
#     hardcoded gamma=1.4 table values 0.5283 / 0.8333,
#   - theta >= theta_max returns an empty-region density of 0.0,
#   - parameterizable beaming exponent kappa.

import numpy as np
import pytest

from pyrpod.plume.RarefiedPlumeGasKinetics import Simons

pytestmark = [pytest.mark.plume, pytest.mark.unit]

R_SPECIFIC = 287.0  # J / (kg K)
T_C = 500.0         # K
P_C = 1.0e5         # N / m^2
R_0 = 0.1           # m
R_FIELD = 2.0       # m


def make_simons(gamma, kappa=None):
    return Simons(gamma, R_SPECIFIC, T_C, P_C, R_0, R_FIELD, kappa=kappa)


def test_throat_ratios_reproduce_gamma_1_4_table_values():
    """The closed forms T*/T_c = 2/(gamma+1), P*/P_c = (2/(gamma+1))^
    (gamma/(gamma-1)) must reproduce the isentropic-table values
    0.8333 / 0.5283 for gamma = 1.4 to 4 decimal places."""
    gamma = 1.4
    t_ratio = 2 / (gamma + 1)
    p_ratio = t_ratio ** (gamma / (gamma - 1))
    assert t_ratio == pytest.approx(0.8333, abs=5e-5)
    assert p_ratio == pytest.approx(0.5283, abs=5e-5)

    # And the class must use them: density/sonic velocity at gamma=1.4
    # match the old hardcoded-table implementation to table precision.
    simons = make_simons(gamma)
    rho_old_table = (P_C * 0.5283) / (R_SPECIFIC * T_C * 0.8333)
    assert simons.get_nozzle_throat_density() == pytest.approx(rho_old_table,
                                                               rel=1e-4)
    a_old_table = np.sqrt(gamma * R_SPECIFIC * T_C * 0.8333)
    assert simons.get_sonic_velocity() == pytest.approx(a_old_table, rel=1e-4)


def test_gamma_2_yields_real_finite_normalization():
    """Regression for the old 'broken when gamma = 2' failure: the
    numerically integrated normalization constant is a plain real,
    positive, finite float for fractional/even kappa alike."""
    simons = make_simons(2.0)
    assert isinstance(simons.A, float)
    assert np.isfinite(simons.A)
    assert simons.A > 0.0


@pytest.mark.parametrize("gamma", [1.4, 5.0 / 3.0, 2.0])
def test_density_zero_beyond_limiting_angle(gamma):
    """theta >= theta_max is a physically empty region: both the decay
    function and the density ratio must return exactly 0.0 (previously
    the cosine went negative and fractional kappa produced complex
    numbers)."""
    simons = make_simons(gamma)
    theta_max = simons.get_limiting_turn_angle()
    for theta in [theta_max, 1.01 * theta_max, 2.0 * theta_max]:
        f = simons.get_plume_angular_density_decay_function(theta)
        n = simons.get_num_density_ratio(theta)
        assert f == 0.0
        assert n == 0.0
        assert not isinstance(f, complex)
        assert not isinstance(n, complex)


def test_density_positive_inside_limiting_angle():
    simons = make_simons(2.0)
    theta_max = simons.get_limiting_turn_angle()
    n = simons.get_num_density_ratio(0.99 * theta_max)
    assert np.isreal(n)
    assert 0.0 < n < simons.get_num_density_ratio(0.0)


def test_default_kappa_is_boyton():
    """Default beaming exponent is Boyton's kappa = 2/(gamma-1)
    [Cai & Wang 2012, Sec. II.C, refs. 22-23]."""
    gamma = 1.4
    simons = make_simons(gamma)
    assert simons.kappa == pytest.approx(2 / (gamma - 1))


@pytest.mark.parametrize("kappa", [2.0, 2.5, 1 / 0.4])
def test_kappa_parameterization(kappa):
    """Custom kappa (e.g. Ashkenas & Sherman's 2, Albini's 1/(gamma-1))
    is used by the decay function: f = cos^kappa at the half angle,
    and f(0) = 1 for every kappa (paper Sec. II.C)."""
    simons = make_simons(1.4, kappa=kappa)
    assert simons.kappa == kappa
    assert simons.get_plume_angular_density_decay_function(0.0) == 1.0
    theta_max = simons.get_limiting_turn_angle()
    theta = 0.5 * theta_max
    expected = np.cos((np.pi / 2) * (theta / theta_max)) ** kappa
    f = simons.get_plume_angular_density_decay_function(theta)
    assert f == pytest.approx(expected, rel=1e-12)


def test_kappa_changes_normalization_constant():
    """A comes from mass-flow continuity over the chosen decay function,
    so it must respond to kappa: a larger kappa concentrates the plume,
    requiring a larger A."""
    a_kappa_2 = make_simons(1.4, kappa=2.0).A
    a_kappa_5 = make_simons(1.4, kappa=5.0).A
    assert a_kappa_5 > a_kappa_2 > 0.0


def test_exit_referenced_density_at_mach_1_equals_throat_referenced():
    """With exit Mach 1 the exit IS the throat, so n/n_0 == n/n_s."""
    simons = make_simons(1.4)
    theta = 0.3
    assert simons.get_num_density_ratio_exit(theta, 1.0) == pytest.approx(
        simons.get_num_density_ratio(theta), rel=1e-12)


def test_exit_referenced_density_scaling():
    """n/n_0 = (n/n_s) * (n_s/n_0) with the isentropic throat-to-exit
    density ratio [(1+(gamma-1)/2*Me^2)/((gamma+1)/2)]^(1/(gamma-1)).
    The throat is denser than a supersonic exit, so the factor is > 1."""
    gamma = 1.4
    exit_mach = 5.0
    simons = make_simons(gamma)
    theta = 0.3
    factor = ((1 + (gamma - 1) / 2 * exit_mach ** 2)
              / ((gamma + 1) / 2)) ** (1 / (gamma - 1))
    assert factor > 1.0
    assert simons.get_num_density_ratio_exit(theta, exit_mach) == pytest.approx(
        simons.get_num_density_ratio(theta) * factor, rel=1e-12)
