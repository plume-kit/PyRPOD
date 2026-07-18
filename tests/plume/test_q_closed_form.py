# ========================
# PyRPOD: tests/plume/test_q_closed_form.py
# ========================
# Verifies the closed form of the special factor Q (Cai & Wang 2012,
# Eq. 9) against a direct truncation of the printed Legendre series.
#
# Eq. 9: Q = cos^2(psi) * [sum_n P_n(sin(psi)sin(eps)) * t^n]^2 with
# t = r/sqrt(X^2+Z^2). The series is the Legendre generating function,
# so Q collapses to X^2/(X^2 + Z^2 - 2*Z*r*sin(eps) + r^2). The series
# converges like t^n, so with the test points restricted to t <= 0.6 a
# 50-term truncation is accurate to ~t^51 ~ 5e-12, and rel=1e-9 both
# passes robustly and would catch any error in the closed form.

import numpy as np
import pytest
from scipy.special import eval_legendre

from pyrpod.plume.RarefiedPlumeGasKinetics import get_Q_full

pytestmark = pytest.mark.plume

N_TERMS = 50


def q_series(r, epsilon, X, Z, n_terms=N_TERMS):
    """Truncated Eq. 9 exactly as printed in the paper."""
    psi = np.arctan2(Z, X)
    x = np.sin(psi) * np.sin(epsilon)
    t = r / np.sqrt(X ** 2 + Z ** 2)
    series = sum(eval_legendre(n, x) * t ** n for n in range(n_terms))
    return np.cos(psi) ** 2 * series ** 2


# (r, epsilon, X, Z) with t = r/sqrt(X^2+Z^2) <= 0.6
SERIES_POINTS = [
    (0.05, 0.0, 1.0, 0.5),
    (0.10, 0.7, 1.0, 0.5),
    (0.10, -1.2, 1.0, 0.5),
    (0.30, 1.5, 0.8, 0.6),
    (0.60, -0.4, 1.0, 0.2),
    (0.05, 0.3, 0.1, 0.05),
    (0.50, 1.0, 2.0, 1.5),
]


@pytest.mark.parametrize("r,epsilon,X,Z", SERIES_POINTS)
def test_closed_form_matches_legendre_series(r, epsilon, X, Z):
    """Closed form == 50-term truncation of the printed Eq. 9 series."""
    assert get_Q_full(r, epsilon, X, Z) == pytest.approx(
        q_series(r, epsilon, X, Z), rel=1e-9)


def test_centerline_reduction():
    """On the centerline (Z = 0), Q reduces to X^2/(X^2 + r^2) for any
    epsilon (needed by the Eq. 21 centerline temperature quadrature)."""
    X = 0.7
    for r in [0.0, 0.05, 0.1]:
        for epsilon in [-1.0, 0.0, 1.3]:
            assert get_Q_full(r, epsilon, X, 0.0) == pytest.approx(
                X ** 2 / (X ** 2 + r ** 2), rel=1e-14)


def test_q_bounded_between_zero_and_one():
    """0 < Q <= 1 everywhere in the integration domain: the denominator
    is X^2 + (Z - r sin(eps))^2 + (r cos(eps))^2 >= X^2. This bound is
    what makes the exp(-S_0^2*(1-Q)) combination overflow-safe, so it
    is asserted over a grid that includes the worst case Z ~ r*sin(eps)."""
    r = np.linspace(0.0, 0.1, 41)
    epsilon = np.linspace(-np.pi / 2, np.pi / 2, 41)
    R, E = np.meshgrid(r, epsilon)
    for X, Z in [(1.0, 0.5), (0.05, 0.08), (0.01, 0.0), (0.3, 3.0)]:
        Q = get_Q_full(R, E, X, Z)
        assert np.all(Q > 0.0)
        assert np.all(Q <= 1.0)
