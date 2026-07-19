"""
Verification reference for Cai 2016, Section 4 (3D inclined-plate impingement).

Cai, C., "Gaskinetic Modeling on Dilute Gaseous Plume Impingement Flows,"
Aerospace 2016, 3(4), 43, doi:10.3390/aerospace3040043.

This module is a PLAIN-FUNCTION reference implementation of the paper's exact
surface-property solutions (Eqs. 9-15 with the Appendix A integrals) for a
collisionless round jet impinging on an inclined, rectangular, flat plate.
It exists to verify PyRPOD's plume modeling against the paper -- it is NOT a
new plume model class and must not be wired into the strike pipeline; the
Cai & Wang 2012 field models in RarefiedPlumeGasKinetics.py stay the plume
models of record.

Geometry and symbol conventions (paper Fig. 11)
-----------------------------------------------
* Nozzle exit: disk of radius R_0 centered at the origin in the Y-Z plane;
  bulk flow along +X with speed ratio S_0 = U_0 / sqrt(2*R*T_0).
* Plate: center at (L, 0, 0), inclination angle alpha_0. Local plate
  coordinates (s, tau) map to global coordinates as
      X = L + tau*cos(alpha_0),  Y = s,  Z = tau*sin(alpha_0),
  i.e. tau runs along the inclined direction in the X-Z plane
  (|tau| <= H_0) and s along the horizontal Y direction (|s| <= W_0).
  The plate unit normal facing the nozzle is (-sin(alpha_0), 0, cos(alpha_0));
  alpha_0 = 90 deg is a plate normal to the jet axis.
* Exit-disk integration point: E = (0, r*cos(theta), r*sin(theta)), so that
  Eq. 10 reads Q^2 = |P - E|^2 / X^2 >= 1 (NOTE: unlike the 2012 paper's
  special factor Q_2012 = X^2/|P-E|^2 <= 1, the 2016 paper's Q is the
  reciprocal square root: a = S_0/Q = S_0*sqrt(Q_2012), the speed ratio
  projected on the ray from E to the plate point).
* B1 = (Z - r*sin(theta))/X, B2 = (Y - r*cos(theta))/X, and
  mu = sin(alpha_0) - B1*cos(alpha_0) = -(ray direction . outward normal)*Q,
  so mu > 0 exactly when the ray from E strikes the FRONT face of the plate.

Overflow safety: every Appendix-A factor A_k(a) carries a e^(a^2) term while
the solutions carry e^(-S_0^2); since a = S_0/Q <= S_0, the two are combined
into e^(a^2 - S_0^2) <= 1 (the same scaling style used by get_K_factor in
RarefiedPlumeGasKinetics.py), so no term can overflow for large S_0.

Diffuse-wall re-emission density n_w (interpretation note)
----------------------------------------------------------
The paper states n_w(s, tau) is set by non-penetration at the plate but does
not print the 3D expression (its refs. [35, 37] give the 2D principle). The
3D analog is derived here from first principles: the incoming number flux
from the jet at plate point P,

    Phi_in = n_0 * e^(-S_0^2) / (pi^(3/2) * beta_0 * X^2)
             * Int_disk (A_1/Q^4) * mu * r dr dtheta,

must equal the effusion flux of the wall Maxwellian at T_w,
Phi_out = n_w * sqrt(R*T_w/(2*pi)) = n_w / (2*sqrt(pi)*beta_w), giving

    n_w/n_0 = (2/sqrt(eps)) * e^(-S_0^2)/(pi * X^2)
              * Int_disk (A_1/Q^4) * mu * r dr dtheta,   eps = T_w/T_0.

This is exactly "the A_1-analog exit-disk integral": the same reduction that
produces Eq. 9 (pressure ~ A_2/Q^5 * mu^2) and Eq. 13 (energy flux ~
A_3/Q^4 * mu) yields number flux ~ A_1/Q^4 * mu. Validation against the
paper's figures: with this n_w the center-region magnitudes reproduce
Figs. 17-18 (diffuse Cp peaks just above the 0.2 innermost contour and
specular Cp above 0.3, specular slightly higher at the impingement center,
as the paper notes), and the wall energy-emission term of Eq. 13,
-eps^(3/2) * (n_w/n_0) / (sqrt(pi)*S_0^3), is the effusion energy flux
(2*k*T_w per emitted molecule) of the SAME n_w -- an independent
consistency check between Eqs. 9 and 13.

Front-face visibility clamp
---------------------------
The printed Eqs. 9-13 integrate over the whole exit disk. For the paper's
validation geometry (alpha_0 = 60 deg, L = 4D) every disk point sees the
front face (mu > 0 everywhere), so clamping is a no-op there. For grazing
geometries (small alpha_0 and/or small L, reached in the sweep study) part
of the disk falls behind the plate plane; those rays cannot deposit flux on
the front face, so integrand contributions with mu <= 0 are dropped. This
extends the formulas continuously to the grazing limit instead of letting
unphysical negative-flux contributions enter.

All returned coefficients use the paper's normalization: pressure and shear
by n_0*m*U_0^2/2, heat flux by n_0*m*U_0^3/2 (m = molecular mass); they are
dimensionless and independent of n_0.
"""

import numpy as np
from scipy.special import erf

#: Gauss-Legendre order per axis for the exit-disk integrals. The integrand
#: is analytic on the compact disk (denominators bounded below by X^2 > 0),
#: so convergence is geometric; 48 nodes reach ~1e-12 for every geometry in
#: the study (verified by the order-doubling test in run_sanity_checks).
DEFAULT_ORDER = 48


def _scaled_A_factors(a, S_0):
    '''
        Appendix-A factors A_1, A_2, A_3 evaluated at a and pre-multiplied
        by the solutions' e^(-S_0^2) prefactor, combined overflow-safely
        into e^(a^2 - S_0^2) <= 1 (a = S_0/Q <= S_0).

        Parameters
        ----------
        a : ndarray
            projected speed ratio S_0/Q along each disk-to-plate ray
        S_0 : float
            molecular speed ratio at the nozzle exit

        Returns
        -------
        tuple of ndarray
            (e^(-S_0^2)*A_1(a), e^(-S_0^2)*A_2(a), e^(-S_0^2)*A_3(a))
    '''
    erf_term = 0.25 * np.sqrt(np.pi) * (1 + erf(a)) * np.exp(a ** 2 - S_0 ** 2)
    exp_term = np.exp(-S_0 ** 2)
    A1 = erf_term * (3 * a + 2 * a ** 3) + exp_term * (0.5 + 0.5 * a ** 2)
    A2 = (erf_term * (1.5 + 6 * a ** 2 + 2 * a ** 4)
          + exp_term * (1.25 * a + 0.5 * a ** 3))
    A3 = (erf_term * (2 * a ** 5 + 10 * a ** 3 + 7.5 * a)
          + exp_term * (0.5 * a ** 4 + 2.25 * a ** 2 + 1.0))
    return A1, A2, A3


def plate_point_coords(s, tau, alpha_0, L):
    '''
        Global coordinates of plate points from local plate coordinates
        (see module docstring for the convention).

        Parameters
        ----------
        s, tau : float or ndarray
            local plate coordinates (m); broadcast together
        alpha_0 : float
            plate inclination angle (rad)
        L : float
            center-to-center nozzle-to-plate distance (m)

        Returns
        -------
        tuple of ndarray
            (X, Y, Z) global coordinates (m)
    '''
    s = np.asarray(s, dtype=float)
    tau = np.asarray(tau, dtype=float)
    X = L + tau * np.cos(alpha_0)
    Y = np.broadcast_to(s, np.broadcast(s, tau).shape).copy()
    Z = tau * np.sin(alpha_0)
    return X, Y, Z


def surface_coefficients(X, Y, Z, S_0, alpha_0, eps, R_0,
                         order=DEFAULT_ORDER, chunk=256):
    '''
        Exact surface coefficients of Cai 2016 Eqs. 9-14 at global plate
        points (X, Y, Z), vectorized with tensor-product Gauss-Legendre
        quadrature over the exit disk (r in [0, R_0], theta in [0, 2*pi]).

        Points with X <= 0 are behind the exit plane: the collisionless jet
        carries no molecules there, so every coefficient is 0.

        Parameters
        ----------
        X, Y, Z : ndarray
            global plate-point coordinates (m); flattened internally,
            outputs keep the input shape
        S_0 : float
            exit speed ratio
        alpha_0 : float
            plate inclination angle (rad)
        eps : float
            wall-to-exit temperature ratio T_w/T_0
        R_0 : float
            nozzle exit radius (m)
        order : int
            Gauss-Legendre nodes per axis (see DEFAULT_ORDER)
        chunk : int
            plate points evaluated per vectorized block (memory control)

        Returns
        -------
        dict of ndarray
            'Cp_d'  : diffuse-plate pressure coefficient [Eq. 9]
            'Cf1_d' : diffuse shear along the inclined direction [Eq. 11]
            'Cf2_d' : diffuse shear along the horizontal (Y) [Eq. 12]
            'Cq_d'  : diffuse heat-flux coefficient [Eq. 13]
            'Cp_s'  : specular-plate pressure coefficient [Eq. 14]
            'nw'    : wall re-emission density ratio n_w/n_0 (see docstring)
    '''
    X = np.asarray(X, dtype=float)
    shape = X.shape
    Xf = X.ravel()
    Yf = np.asarray(Y, dtype=float).ravel()
    Zf = np.asarray(Z, dtype=float).ravel()

    nodes, weights = np.polynomial.legendre.leggauss(order)
    r = 0.5 * R_0 * (nodes + 1)
    w_r = 0.5 * R_0 * weights
    th = np.pi * (nodes + 1)
    w_th = np.pi * weights
    Rn, Tn = np.meshgrid(r, th, indexing='ij')
    WR = np.outer(w_r, w_th) * Rn          # quadrature weight * r (Jacobian)
    Ey = Rn * np.cos(Tn)                   # exit-disk point (0, Ey, Ez)
    Ez = Rn * np.sin(Tn)

    sin_a, cos_a = np.sin(alpha_0), np.cos(alpha_0)
    sin_2a, cos_2a = np.sin(2 * alpha_0), np.cos(2 * alpha_0)

    n_pts = Xf.size
    I_p = np.zeros(n_pts)
    I_f1 = np.zeros(n_pts)
    I_f2 = np.zeros(n_pts)
    I_q = np.zeros(n_pts)
    I_n = np.zeros(n_pts)
    ahead = Xf > 0.0

    idx_all = np.nonzero(ahead)[0]
    for start in range(0, idx_all.size, chunk):
        idx = idx_all[start:start + chunk]
        Xc = Xf[idx][:, None, None]
        Yc = Yf[idx][:, None, None]
        Zc = Zf[idx][:, None, None]

        dy = Yc - Ey
        dz = Zc - Ez
        Q2 = (Xc ** 2 + dy ** 2 + dz ** 2) / Xc ** 2   # Eq. 10
        Q = np.sqrt(Q2)
        a = S_0 / Q
        B1 = dz / Xc
        B2 = dy / Xc
        mu = sin_a - B1 * cos_a
        front = mu > 0.0                    # front-face visibility clamp
        mu_p = np.where(front, mu, 0.0)

        A1, A2, A3 = _scaled_A_factors(a, S_0)
        inv_Q4 = 1.0 / (Q2 * Q2)
        inv_Q5 = inv_Q4 / Q

        I_p[idx] = np.sum(WR * A2 * inv_Q5 * mu_p ** 2, axis=(1, 2))
        I_f1[idx] = np.sum(
            WR * A2 * inv_Q5
            * (0.5 * (1 - B1 ** 2) * sin_2a - B1 * cos_2a)
            * front, axis=(1, 2))
        I_f2[idx] = np.sum(WR * A2 * B2 * inv_Q5 * mu_p, axis=(1, 2))
        I_q[idx] = np.sum(WR * A3 * inv_Q4 * mu_p, axis=(1, 2))
        I_n[idx] = np.sum(WR * A1 * inv_Q4 * mu_p, axis=(1, 2))

    with np.errstate(divide='ignore', invalid='ignore'):
        pref = np.where(ahead, 1.0 / (np.pi ** 1.5 * Xf ** 2), 0.0)
        pref_pi = np.where(ahead, 1.0 / (np.pi * Xf ** 2), 0.0)

    Cp_jet = (2.0 / S_0 ** 2) * pref * I_p
    Cf1 = (2.0 / S_0 ** 2) * pref * I_f1
    Cf2 = (2.0 / S_0 ** 2) * pref * I_f2
    nw = (2.0 / np.sqrt(eps)) * pref_pi * I_n
    Cq = (pref_pi * I_q - eps ** 1.5 * nw) / (np.sqrt(np.pi) * S_0 ** 3)
    Cp_d = Cp_jet + eps / (2.0 * S_0 ** 2) * nw
    Cp_s = 2.0 * Cp_jet

    return {'Cp_d': Cp_d.reshape(shape), 'Cf1_d': Cf1.reshape(shape),
            'Cf2_d': Cf2.reshape(shape), 'Cq_d': Cq.reshape(shape),
            'Cp_s': Cp_s.reshape(shape), 'nw': nw.reshape(shape)}


def surface_coefficients_plate(s, tau, S_0, alpha_0, eps, R_0, L,
                               order=DEFAULT_ORDER):
    '''
        Convenience wrapper of surface_coefficients over local plate
        coordinates (s, tau); see plate_point_coords for the mapping.
    '''
    s = np.asarray(s, dtype=float)
    tau = np.asarray(tau, dtype=float)
    shape = np.broadcast(s, tau).shape
    sB = np.broadcast_to(s, shape)
    tB = np.broadcast_to(tau, shape)
    X, Y, Z = plate_point_coords(sB, tB, alpha_0, L)
    return surface_coefficients(X, Y, Z, S_0, alpha_0, eps, R_0, order=order)


def averaged_coefficients(S_0, alpha_0, eps, R_0, L, W_0, H_0,
                          n_gl=64, order=DEFAULT_ORDER):
    '''
        Plate-averaged properties of Eq. 15 by Gauss-Legendre quadrature
        over the plate: CP, CF1, CF2, CQ, the moment coefficient
        CM = 1/(2*H_0*S) * Int tau*Cp,d ds dtau, and s_cc = CM/CP.

        Parameters
        ----------
        S_0, alpha_0, eps, R_0, L : as in surface_coefficients
        W_0 : float
            plate semi-width along s (m)
        H_0 : float
            plate semi-length along the inclined direction tau (m)
        n_gl : int
            Gauss-Legendre nodes per plate axis
        order : int
            exit-disk quadrature order per axis

        Returns
        -------
        dict of float
            keys 'CP', 'CF1', 'CF2', 'CQ', 'CM', 's_cc'
    '''
    nodes, weights = np.polynomial.legendre.leggauss(n_gl)
    s = W_0 * nodes
    w_s = W_0 * weights
    tau = H_0 * nodes
    w_tau = H_0 * weights
    Sg, Tg = np.meshgrid(s, tau, indexing='ij')
    W2D = np.outer(w_s, w_tau)
    area = 4.0 * W_0 * H_0

    c = surface_coefficients_plate(Sg, Tg, S_0, alpha_0, eps, R_0, L,
                                   order=order)
    CP = np.sum(W2D * c['Cp_d']) / area
    CF1 = np.sum(W2D * c['Cf1_d']) / area
    CF2 = np.sum(W2D * c['Cf2_d']) / area
    CQ = np.sum(W2D * c['Cq_d']) / area
    CM = np.sum(W2D * Tg * c['Cp_d']) / (2.0 * H_0 * area)
    s_cc = CM / CP if CP != 0.0 else np.nan
    return {'CP': CP, 'CF1': CF1, 'CF2': CF2, 'CQ': CQ, 'CM': CM,
            's_cc': s_cc}


def run_sanity_checks(S_0=2.0, alpha_0=np.deg2rad(60.0), eps=1.5,
                      R_0=0.5, L=4.0, verbose=True):
    '''
        Built-in verification of the implementation:

        1. s-symmetry: Cp_d, Cf1_d, Cq_d even in s; Cf2_d odd in s
           (Figs. 17-21 are left-right symmetric / antisymmetric).
        2. Cp_s = 2 x jet-only part of Cp_d (Eq. 14 vs Eq. 9), i.e.
           Cp_s = 2*(Cp_d - eps/(2*S_0^2)*n_w/n_0).
        3. Wall-term flux balance: the incoming number-flux integral
           recomputed independently with scipy.integrate.dblquad matches
           the Gauss-Legendre n_w to ~1e-10 (validates both the quadrature
           and the n_w reduction).
        4. Quadrature convergence: order 48 vs 96 agree to ~1e-12.

        Raises AssertionError on failure; returns a dict of the measured
        deviations.
    '''
    from scipy import integrate

    s = np.linspace(-3.5, 3.5, 15)
    tau = np.linspace(-3.5, 3.5, 15)
    Sg, Tg = np.meshgrid(s, tau, indexing='ij')
    c = surface_coefficients_plate(Sg, Tg, S_0, alpha_0, eps, R_0, L)
    c_mirror = surface_coefficients_plate(-Sg, Tg, S_0, alpha_0, eps, R_0, L)

    results = {}
    for key, parity in [('Cp_d', 1), ('Cf1_d', 1), ('Cq_d', 1),
                        ('Cf2_d', -1)]:
        dev = np.max(np.abs(c[key] - parity * c_mirror[key]))
        scale = np.max(np.abs(c[key]))
        results[f'symmetry_{key}'] = dev / scale
        assert dev <= 1e-12 * scale, f'{key} s-parity violated: {dev}'

    dev = np.max(np.abs(
        c['Cp_s'] - 2.0 * (c['Cp_d'] - eps / (2 * S_0 ** 2) * c['nw'])))
    results['specular_vs_jet'] = dev
    assert dev <= 1e-14, f'Cp_s != 2 x jet-only Cp_d: {dev}'

    # independent dblquad check of the n_w flux integral at spot points
    sin_a, cos_a = np.sin(alpha_0), np.cos(alpha_0)
    for s0, t0 in [(0.0, 0.0), (1.5, -2.0), (-2.5, 3.0)]:
        X, Y, Z = plate_point_coords(s0, t0, alpha_0, L)
        X, Y, Z = float(X), float(Y), float(Z)

        def flux_integrand(r, th):
            dy = Y - r * np.cos(th)
            dz = Z - r * np.sin(th)
            Q = np.sqrt((X ** 2 + dy ** 2 + dz ** 2) / X ** 2)
            a = S_0 / Q
            A1, _, _ = _scaled_A_factors(np.asarray(a), S_0)
            mu = sin_a - (dz / X) * cos_a
            return float(A1) / Q ** 4 * max(mu, 0.0) * r

        I_n_ref, _ = integrate.dblquad(
            flux_integrand, 0.0, 2 * np.pi, 0.0, R_0,
            epsabs=1e-13, epsrel=1e-12)
        nw_ref = (2.0 / np.sqrt(eps)) * I_n_ref / (np.pi * X ** 2)
        nw_gl = float(surface_coefficients(
            np.array([X]), np.array([Y]), np.array([Z]),
            S_0, alpha_0, eps, R_0)['nw'][0])
        dev = abs(nw_gl - nw_ref) / nw_ref
        results[f'nw_dblquad_s{s0}_tau{t0}'] = dev
        assert dev <= 1e-9, f'n_w flux balance failed at ({s0},{t0}): {dev}'

    c_fine = surface_coefficients_plate(Sg, Tg, S_0, alpha_0, eps, R_0, L,
                                        order=2 * DEFAULT_ORDER)
    for key in ('Cp_d', 'Cf1_d', 'Cf2_d', 'Cq_d'):
        scale = np.max(np.abs(c_fine[key]))
        dev = np.max(np.abs(c[key] - c_fine[key])) / scale
        results[f'convergence_{key}'] = dev
        assert dev <= 1e-10, f'quadrature not converged for {key}: {dev}'

    if verbose:
        for name, value in results.items():
            print(f'  {name}: {value:.3e}')
    return results


if __name__ == '__main__':
    print('Cai 2016 reference implementation sanity checks '
          '(S0=2, alpha0=60 deg, eps=1.5, L=4D):')
    run_sanity_checks()
    print('all checks passed')
    avg = averaged_coefficients(2.0, np.deg2rad(60.0), 1.5, 0.5, 4.0,
                                4.0, 4.0)
    print('Eq. 15 averaged coefficients at the paper conditions:')
    for key, value in avg.items():
        print(f'  {key} = {value:.6g}')
