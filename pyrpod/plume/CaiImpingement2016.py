"""
Verification reference for Cai 2016 (inclined-plate plume impingement).

Cai, C., "Gaskinetic Modeling on Dilute Gaseous Plume Impingement Flows,"
Aerospace 2016, 3(4), 43, doi:10.3390/aerospace3040043.

This module is a PLAIN-FUNCTION reference implementation of the paper's exact
solutions -- the single source of truth for every Cai-2016 verification
figure in tests/plume. It covers:

* Section 4 surface properties: diffuse-plate Cp/Cf1/Cf2/Cq (Eqs. 9-13),
  specular Cp (Eq. 14), Eq. 15 plate averages (Figs. 17-21);
* Section 3, the 2D slot jet on an inclined planar plate: surface
  coefficients Cp,d/Cf,d/Cq,d and Cp,s (Eqs. 2-4, 8; Figs. 7-10) and the
  combined jet + plate flowfield temperature (Figs. 5-6);
* Section 4 flowfield pressure in the vertical Y = 0 plane with the plate
  contribution, diffuse and specular (Figs. 15-16).

The free-jet field factors of the 3D solutions are imported from
pyrpod/plume/RarefiedPlumeGasKinetics.py (the Cai & Wang 2012 plume model
of record) rather than re-derived, so the two implementations can never
drift apart. This module exists to verify PyRPOD's plume modeling against
the paper -- it is NOT a new plume model class and must not be wired into
the strike pipeline.

Section-3 validation geometry (interpretation note): this paper does not
print the 2D case's numbers (they come from its ref. [39], Cai & He 2016);
they are read off Figs. 1 and 5-10: slot height 2H, center-to-center
distance L = 4*(2H), plate semi-width W = 5*(2H), Tw/T0 = 1.5, with
(S0, alpha0) per figure legend. The plotted axes are X/(2H) and s/(2H).

Flowfield figures (5-6, 15-16) evaluate the analytic solutions on the
whole X > 0 half-plane with no plate shadowing, exactly as the paper's
contours do (Fig. 6's specular temperature field is symmetric about the
plate line only under this convention); the diffuse plate emits from its
front face only, so behind-plate points carry the jet-only continuation.
Specular-plate effects use the paper's virtual-nozzle construction; the
virtual exit is built by literal mirror reflection of the real exit about
the plate plane, which reproduces Eq. 7 (2D) and the printed 3D virtual
position (L(1 - cos 2a0), 0, -L sin 2a0). The virtual DRIFT is the mirror
image U0*(cos 2a0, 0, sin 2a0); the paper's prose sign
(-U0 cos 2a0, ..., -U0 sin 2a0) is the velocity-space DOMAIN VERTEX -U0'
of Fig. 14, not the drift (same resolution style as the 2012 Eq. 7 typo
note in RarefiedPlumeGasKinetics).

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

from pyrpod.plume.RarefiedPlumeGasKinetics import (
    get_K_factor,
    get_M_factor,
    get_N_factor,
    get_Q_full,
)

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


# ---------------------------------------------------------------------------
# Section 3: 2D slot jet impinging on an inclined planar plate (Eqs. 1-8)
# ---------------------------------------------------------------------------

def _scaled_planar_factors(a, S):
    '''
        e^(-S^2)-scaled polar-velocity wedge moments e^(a^2) * I_k(a) of a
        drifting Maxwellian in 2D, k = 1, 2, 3, where
        I_k(a) = Int_0^inf t^k e^(-(t-a)^2) dt; k = 2 and 3 reproduce the
        Appendix A_0 and A_1. Here a = S*cos(theta - drift angle) may be
        NEGATIVE (rays opposed to the drift); a^2 <= S^2 keeps every
        exponential bounded and (1 + erf(a)) -> 0 kills opposed rays.

        Returns (E1, A0, A1), each pre-multiplied by e^(-S^2).
    '''
    erf_term = (1 + erf(a)) * np.exp(a ** 2 - S ** 2)
    exp_term = np.exp(-S ** 2)
    sq = np.sqrt(np.pi)
    E1 = 0.5 * exp_term + 0.5 * sq * a * erf_term
    A0 = 0.25 * sq * (1 + 2 * a ** 2) * erf_term + 0.5 * a * exp_term
    A1 = (0.25 * sq * (3 * a + 2 * a ** 3) * erf_term
          + (0.5 + 0.5 * a ** 2) * exp_term)
    return E1, A0, A1


def _scaled_G_factor(a, S):
    '''
        e^(-S^2)-scaled energy-flux integrand of Eq. 4,
        G(a) = (sqrt(pi)/2)(2 + 7a^2 + 2a^4) e^(a^2) [1 + erf(a)] + 3a + a^3.
        Verified to equal 2 e^(a^2) (I_4 + I_2/2): the planar c^3 moment
        plus the out-of-plane w^2 energy carried by the number flux.
    '''
    erf_term = (1 + erf(a)) * np.exp(a ** 2 - S ** 2)
    return (0.5 * np.sqrt(np.pi) * (2 + 7 * a ** 2 + 2 * a ** 4) * erf_term
            + (3 * a + a ** 3) * np.exp(-S ** 2))


def planar_plate_point_coords(s, alpha_0, L):
    '''2D plate point (X, Y) = (L + s cos(alpha_0), s sin(alpha_0)).'''
    s = np.asarray(s, dtype=float)
    return L + s * np.cos(alpha_0), s * np.sin(alpha_0)


def planar_surface_coefficients(s, S_0, alpha_0, eps, H, L,
                                order=DEFAULT_ORDER):
    '''
        Exact 2D surface coefficients of Cai 2016 Eqs. 2-4 and 8 at plate
        positions s (distance from the plate center along the plate).

        The wedge subtended by the slot exit (x = 0, y in [-H, H]) at the
        plate point (X, Y) spans theta in [atan2(Y - H, X), atan2(Y + H, X)]
        (Eq. 1); a = S_0 cos(theta). The diffuse-wall density n_w(s) follows
        from non-penetration with the A_0 number-flux integral,
        n_w/n_0 = 2/sqrt(pi*eps) * e^(-S_0^2) *
                  Int A_0(a) sin(alpha_0 - theta) dtheta,
        the 2D analog of the 3D A_1-integral (module docstring). Front-face
        visibility clamps sin(alpha_0 - theta) <= 0 contributions, and
        plate points behind the exit plane (X <= 0) carry zero load.

        Returns dict with 'Cp_d', 'Cf_d', 'Cq_d', 'Cp_s', 'nw'.
    '''
    s = np.asarray(s, dtype=float)
    shape = s.shape
    X, Y = planar_plate_point_coords(s.ravel(), alpha_0, L)
    ahead = X > 0.0

    nodes, wts = np.polynomial.legendre.leggauss(order)
    Xa, Ya = X[ahead][:, None], Y[ahead][:, None]
    th1 = np.arctan2(Ya - H, Xa)
    th2 = np.arctan2(Ya + H, Xa)
    mid, half = 0.5 * (th1 + th2), 0.5 * (th2 - th1)
    theta = mid + half * nodes[None, :]
    w = half * wts[None, :]

    a = S_0 * np.cos(theta)
    _, A0s, A1s = _scaled_planar_factors(a, S_0)
    Gs = _scaled_G_factor(a, S_0)
    mu = np.sin(alpha_0 - theta)
    front = mu > 0.0
    mu_p = np.where(front, mu, 0.0)

    I_p = np.sum(w * A1s * mu_p ** 2, axis=1)
    I_f = np.sum(w * A1s * np.sin(2 * alpha_0 - 2 * theta) * front, axis=1)
    I_q = np.sum(w * Gs * mu_p, axis=1)
    I_n = np.sum(w * A0s * mu_p, axis=1)

    def expand(vals):
        full = np.zeros(X.size)
        full[ahead] = vals
        return full.reshape(shape)

    nw = expand(2.0 / np.sqrt(np.pi * eps) * I_n)
    Cp_jet = expand(2.0 / (np.pi * S_0 ** 2) * I_p)
    Cf_d = expand(1.0 / (np.pi * S_0 ** 2) * I_f)
    Cq_d = (expand(I_q / (2.0 * np.pi * S_0 ** 3))
            - eps ** 1.5 * nw / (np.sqrt(np.pi) * S_0 ** 3))
    return {'Cp_d': Cp_jet + eps / (2.0 * S_0 ** 2) * nw,
            'Cf_d': Cf_d, 'Cq_d': Cq_d, 'Cp_s': 2.0 * Cp_jet, 'nw': nw}


def _mirror_about_plate_2d(p, alpha_0, L):
    '''Mirror 2D point(s) about the plate line through (L, 0) at alpha_0.'''
    n_hat = np.array([-np.sin(alpha_0), np.cos(alpha_0)])
    p = np.asarray(p, dtype=float)
    d = (p[..., 0] - L) * n_hat[0] + p[..., 1] * n_hat[1]
    return p - 2.0 * d[..., None] * n_hat


def planar_flowfield(X, Y, S_0, alpha_0, eps, H, L, W, plate='diffuse',
                     order=DEFAULT_ORDER, nw_grid=1024):
    '''
        Combined 2D flowfield moments at points (X, Y) for the Section-3
        problem: the free slot jet plus either the diffuse-plate wall
        emission (Fig. 5) or the specular virtual nozzle (Fig. 6);
        plate=None gives the free jet alone (used by the sanity checks).

        Populations are combined by raw moments (number, momentum, energy
        including the out-of-plane thermal energy at each population's own
        temperature); T/T0 = (2/3) beta_0^2 (M2/n - V^2) + 0 (the 1/3
        out-of-plane share is inside M2). Diffuse emission integrates over
        arrival directions theta with the wall density n_w(s(theta)) at the
        ray-plate intersection, interpolated from a dense s grid, so finite
        plate edges are handled naturally (rays missing the plate carry
        nothing) and only front-face emission counts. The specular virtual
        exit is the literal mirror of the real exit with mirrored drift
        (module docstring). No shadowing is modeled (matching the paper's
        contours); points with X <= 0 return NaN.

        Returns dict 'n' (n/n_0), 'Ux', 'Uy' (times sqrt(beta_0)),
        'T' (T/T_0).
    '''
    X = np.asarray(X, dtype=float)
    shape = X.shape
    Xf = X.ravel()
    Yf = np.asarray(Y, dtype=float).ravel()
    valid = Xf > 0.0
    Xa, Ya = Xf[valid][:, None], Yf[valid][:, None]

    nodes, wts = np.polynomial.legendre.leggauss(order)
    N = np.zeros(Xa.size)
    MX = np.zeros(Xa.size)
    MY = np.zeros(Xa.size)
    M2 = np.zeros(Xa.size)

    def add_population(th_lo, span, drift_angle, S, beta_ratio,
                       n_ref=None):
        '''Wedge population moments; n_ref = None means constant n_0.'''
        nonlocal N, MX, MY, M2
        theta = th_lo + span * 0.5 * (nodes[None, :] + 1.0)
        w = span * 0.5 * wts[None, :]
        a = S * np.cos(theta - drift_angle)
        E1, A0s, _A1s = _scaled_planar_factors(a, S)
        dens = 1.0 if n_ref is None else n_ref
        dN = np.sum(w * dens * E1, axis=1) / np.pi
        N += dN
        MX += np.sum(w * dens * A0s * np.cos(theta), axis=1) \
            / np.pi * beta_ratio
        MY += np.sum(w * dens * A0s * np.sin(theta), axis=1) \
            / np.pi * beta_ratio
        M2 += np.sum(w * dens * _A1s, axis=1) / np.pi * beta_ratio ** 2 \
            + dN * beta_ratio ** 2 / 2.0

    # free jet: wedge subtended by the slot exit, drift along +x
    th1 = np.arctan2(Ya - H, Xa)
    th2 = np.arctan2(Ya + H, Xa)
    add_population(th1, th2 - th1, 0.0, S_0, 1.0)

    if plate == 'diffuse':
        s_grid = np.linspace(-W, W, nw_grid)
        nw_vals = planar_surface_coefficients(s_grid, S_0, alpha_0, eps,
                                              H, L, order=order)['nw']
        t_hat = np.array([np.cos(alpha_0), np.sin(alpha_0)])
        n_hat = np.array([-np.sin(alpha_0), np.cos(alpha_0)])
        ends = np.array([planar_plate_point_coords(sgn * W, alpha_0, L)
                         for sgn in (-1.0, 1.0)])
        ang = np.arctan2(Ya - ends[:, 1], Xa - ends[:, 0])
        span = np.mod(ang[:, 1:2] - ang[:, 0:1] + np.pi, 2 * np.pi) - np.pi
        lo = np.where(span >= 0, ang[:, 0:1], ang[:, 1:2])
        span = np.abs(span)
        theta = lo + span * 0.5 * (nodes[None, :] + 1.0)
        w = span * 0.5 * wts[None, :]
        ct, st = np.cos(theta), np.sin(theta)
        # ray-plate intersection: P = plate(s) + u * theta_hat, u > 0
        denom = t_hat[0] * st - t_hat[1] * ct
        rx, ry = Xa - L, Ya
        with np.errstate(divide='ignore', invalid='ignore'):
            s_hit = (rx * st - ry * ct) / denom
            u_hit = (t_hat[0] * ry - t_hat[1] * rx) / denom
        emitting = (u_hit > 0.0) & (ct * n_hat[0] + st * n_hat[1] > 0.0)
        nw_at = np.interp(s_hit, s_grid, nw_vals, left=0.0, right=0.0)
        nw_at = np.where(emitting & np.isfinite(s_hit), nw_at, 0.0)
        # resting wall Maxwellian: E1 = 1/2, A0 = sqrt(pi)/4, A1 = 1/2
        sqe = np.sqrt(eps)
        dN = np.sum(w * nw_at, axis=1) * 0.5 / np.pi
        N += dN
        MX += np.sum(w * nw_at * ct, axis=1) \
            * (np.sqrt(np.pi) / 4.0) / np.pi * sqe
        MY += np.sum(w * nw_at * st, axis=1) \
            * (np.sqrt(np.pi) / 4.0) / np.pi * sqe
        M2 += np.sum(w * nw_at, axis=1) * 0.5 / np.pi * eps \
            + dN * eps / 2.0
    elif plate == 'specular':
        exit_pts = np.array([[0.0, -H], [0.0, H]])
        mirrored = _mirror_about_plate_2d(exit_pts, alpha_0, L)
        ang = np.arctan2(Ya - mirrored[:, 1], Xa - mirrored[:, 0])
        span = np.mod(ang[:, 1:2] - ang[:, 0:1] + np.pi, 2 * np.pi) - np.pi
        lo = np.where(span >= 0, ang[:, 0:1], ang[:, 1:2])
        add_population(lo, np.abs(span), 2.0 * alpha_0, S_0, 1.0)
    elif plate is not None:
        raise ValueError(f'unknown plate treatment {plate!r}')

    with np.errstate(divide='ignore', invalid='ignore'):
        Ux = MX / N
        Uy = MY / N
        T = (2.0 / 3.0) * (M2 / N - Ux ** 2 - Uy ** 2)

    def expand(vals):
        full = np.full(Xf.size, np.nan)
        full[valid] = vals
        return full.reshape(shape)

    return {'n': expand(N), 'Ux': expand(Ux), 'Uy': expand(Uy),
            'T': expand(T)}


# ---------------------------------------------------------------------------
# Section 4 flowfield pressure in the Y = 0 plane (Figs. 15-16)
# ---------------------------------------------------------------------------

def _plate_emission_moments(Xa, Za, Px, Py, Pz, nw, wA, alpha_0, eps,
                            chunk=128):
    '''
        Raw-moment contributions of the diffuse-wall emission at field
        points (Xa, 0, Za): solid-angle integrals over plate nodes
        (Px, Py, Pz) carrying weights wA and wall densities nw, emitting
        half-space Maxwellians at T_w from the front face only
        (cos(xi) > 0). Per solid angle: dn = n_w/(4 pi) domega,
        d(nV) = n_w sqrt(eps)/(2 pi^(3/2)) domega along the ray, and
        dM2 = 3 n_w eps/(8 pi) domega (the resting-Maxwellian I_2, I_3,
        I_4 moments; full-sphere limits n_w and 3 n_w R T_w check out).

        Returns (dN, dMx, dMy, dMz, dM2) normalized like the jet moments.
    '''
    n_hat = np.array([-np.sin(alpha_0), 0.0, np.cos(alpha_0)])
    npts = Xa.size
    dN = np.zeros(npts)
    dMx = np.zeros(npts)
    dMy = np.zeros(npts)
    dMz = np.zeros(npts)
    dM2 = np.zeros(npts)
    mom = np.sqrt(eps) / (2.0 * np.pi ** 1.5)
    for start in range(0, npts, chunk):
        sl = slice(start, min(start + chunk, npts))
        dx = Xa[sl][:, None] - Px[None, :]
        dy = -Py[None, :] * np.ones((Xa[sl].size, 1))
        dz = Za[sl][:, None] - Pz[None, :]
        d2 = dx ** 2 + dy ** 2 + dz ** 2
        d = np.sqrt(d2)
        cos_xi = (dx * n_hat[0] + dy * n_hat[1] + dz * n_hat[2]) / d
        cos_xi = np.clip(cos_xi, 0.0, None)   # front-face emission only
        dodd = nw[None, :] * cos_xi / d2 * wA[None, :]
        dN[sl] = np.sum(dodd, axis=1) / (4.0 * np.pi)
        dMx[sl] = mom * np.sum(dodd * dx / d, axis=1)
        dMy[sl] = mom * np.sum(dodd * dy / d, axis=1)
        dMz[sl] = mom * np.sum(dodd * dz / d, axis=1)
        dM2[sl] = 3.0 * eps / (8.0 * np.pi) * np.sum(dodd, axis=1)
    return dN, dMx, dMy, dMz, dM2


def _jet_moments_3d(x_loc, rho_loc, S_0, R_0, order=DEFAULT_ORDER,
                    chunk=256):
    '''
        Raw moments (n/n_0, U*sqrt(beta_0), W*sqrt(beta_0),
        M2 = n <c^2> beta_0^2 / n_0) of the free round jet at local
        coordinates (x_loc downstream of the exit, rho_loc off-axis),
        evaluated with the Cai & Wang 2012 exit-disk integrals via the
        K/M/N factors imported from RarefiedPlumeGasKinetics (single
        source of truth). Zero for x_loc <= 0 (no backflow in the model).
    '''
    x_loc = np.asarray(x_loc, dtype=float).ravel()
    rho_loc = np.asarray(rho_loc, dtype=float).ravel()

    nodes, weights = np.polynomial.legendre.leggauss(order)
    r = 0.5 * R_0 * (nodes + 1)
    w_r = 0.5 * R_0 * weights
    epsn = 0.5 * np.pi * nodes
    w_eps = 0.5 * np.pi * weights
    Rn, En = np.meshgrid(r, epsn, indexing='ij')
    W2D = np.outer(w_r, w_eps)
    sinE = np.sin(En)

    n = np.zeros(x_loc.size)
    U = np.zeros(x_loc.size)
    W = np.zeros(x_loc.size)
    M2 = np.zeros(x_loc.size)
    idx_all = np.nonzero(x_loc > 0.0)[0]
    for start in range(0, idx_all.size, chunk):
        idx = idx_all[start:start + chunk]
        Xc = x_loc[idx][:, None, None]
        Zc = rho_loc[idx][:, None, None]
        Q = get_Q_full(Rn, En, Xc, Zc)
        Kf = get_K_factor(Q, S_0)
        Mf = get_M_factor(Q, S_0)
        Nf = get_N_factor(Q, S_0)
        I_K = np.sum(W2D * Rn * Kf, axis=(1, 2))
        I_M = np.sum(W2D * Rn * Mf, axis=(1, 2))
        I_W = np.sum(W2D * (Zc - Rn * sinE) * Rn * Mf, axis=(1, 2))
        I_N = np.sum(W2D * Rn * Nf, axis=(1, 2))
        n[idx] = I_K / (np.pi ** 1.5 * x_loc[idx] ** 2)
        U[idx] = I_M / I_K
        W[idx] = I_W / (x_loc[idx] * I_K)
        T = (-(2.0 / 3.0) * (U[idx] ** 2 + W[idx] ** 2)
             + (4.0 / 3.0) * I_N / I_K)
        M2[idx] = n[idx] * (1.5 * T + U[idx] ** 2 + W[idx] ** 2)
    return n, U, W, M2


def flowfield_pressure_plane(X, Z, S_0, alpha_0, eps, R_0, L, W_0, H_0,
                             plate='diffuse', order=DEFAULT_ORDER,
                             plate_order=48, chunk=128):
    '''
        Static-pressure field p/p_0 = (n/n_0)(T/T_0) in the vertical Y = 0
        plane for the Section-4 3D impingement problem (Figs. 15-16),
        normalized by the exit static pressure p_0 = n_0 k T_0. The local
        temperature enters explicitly (the 2012 paper warns n*k*T_0 is
        invalid), assembled from the raw moments of the populations:

        * the free round jet (2012 exit-disk integrals, see
          _jet_moments_3d);
        * plate='diffuse': the wall re-emission -- a solid-angle integral
          over the plate of half-space Maxwellians with the Eq.-9 n_w(s,
          tau) at the plate's own Gauss-Legendre nodes (front-face
          emission only, cos(xi) > 0);
        * plate='specular': the virtual nozzle mirrored about the plate
          plane with mirrored drift (module docstring), evaluated with the
          same jet integrals in the virtual frame;
        * plate=None: free jet alone (sanity checks).

        No shadowing is modeled, matching the paper's contour convention;
        X <= 0 returns NaN.

        Returns dict 'n', 'T', 'p' (all normalized), shaped like X.
    '''
    X = np.asarray(X, dtype=float)
    shape = X.shape
    Xf = X.ravel()
    Zf = np.asarray(Z, dtype=float).ravel()
    valid = Xf > 0.0
    Xa, Za = Xf[valid], Zf[valid]
    npts = Xa.size

    # free jet in the global frame (axis +x, exit at the origin)
    n1, U1, W1, M21 = _jet_moments_3d(Xa, np.abs(Za), S_0, R_0, order=order)
    N = n1.copy()
    MXv = n1 * U1
    MYv = np.zeros(npts)
    MZv = n1 * W1 * np.sign(Za)
    M2 = M21.copy()

    if plate == 'diffuse':
        nodes, wts = np.polynomial.legendre.leggauss(plate_order)
        s_n = W_0 * nodes
        w_s = W_0 * wts
        t_n = H_0 * nodes
        w_t = H_0 * wts
        Sg, Tg = np.meshgrid(s_n, t_n, indexing='ij')
        wA = np.outer(w_s, w_t).ravel()
        Px, Py, Pz = plate_point_coords(Sg.ravel(), Tg.ravel(), alpha_0, L)
        nw = surface_coefficients(Px, Py, Pz, S_0, alpha_0, eps, R_0,
                                  order=order)['nw']
        dN, dMx, dMy, dMz, dM2 = _plate_emission_moments(
            Xa, Za, Px, Py, Pz, nw, wA, alpha_0, eps, chunk=chunk)
        N += dN
        MXv += dMx
        MYv += dMy
        MZv += dMz
        M2 += dM2
    elif plate == 'specular':
        origin_v = np.array([L * (1.0 - np.cos(2 * alpha_0)), 0.0,
                             -L * np.sin(2 * alpha_0)])
        axis_v = np.array([np.cos(2 * alpha_0), 0.0, np.sin(2 * alpha_0)])
        rx = Xa - origin_v[0]
        rz = Za - origin_v[2]
        x_loc = rx * axis_v[0] + rz * axis_v[2]
        px = rx - x_loc * axis_v[0]
        pz = rz - x_loc * axis_v[2]
        rho = np.hypot(px, pz)
        n2, U2, W2, M22 = _jet_moments_3d(x_loc, rho, S_0, R_0, order=order)
        safe = rho > 1e-12
        rhx = np.where(safe, px / np.where(safe, rho, 1.0), 0.0)
        rhz = np.where(safe, pz / np.where(safe, rho, 1.0), 0.0)
        N += n2
        MXv += n2 * (U2 * axis_v[0] + W2 * rhx)
        MZv += n2 * (U2 * axis_v[2] + W2 * rhz)
        M2 += M22
    elif plate is not None:
        raise ValueError(f'unknown plate treatment {plate!r}')

    with np.errstate(divide='ignore', invalid='ignore'):
        Vx = MXv / N
        Vy = MYv / N
        Vz = MZv / N
        T = (2.0 / 3.0) * (M2 / N - Vx ** 2 - Vy ** 2 - Vz ** 2)
        p = N * T

    def expand(vals):
        full = np.full(Xf.size, np.nan)
        full[valid] = vals
        return full.reshape(shape)

    return {'n': expand(N), 'T': expand(T), 'p': expand(p)}


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


def run_planar_sanity_checks(S_0=2.0, alpha_0=np.deg2rad(60.0), eps=1.5,
                             H=0.5, L=4.0, W=5.0, verbose=True):
    '''
        Verification of the Section-3 (2D planar) implementation:

        1. alpha_0 = 90 deg parity: Cp,d/Cq,d/n_w even and Cf,d odd in s
           (the normal-plate configuration is mirror-symmetric).
        2. Cp,s = 2 x jet-only part of Cp,d (Eq. 8 vs Eq. 2).
        3. n_w flux balance against an independent scipy.integrate.quad
           evaluation of the A_0 number-flux integral.
        4. Diffuse half-space closure: just in front of the plate center
           the wall-emission population fills a half-space, so its density
           tends to n_w(0)/2 (checked via flowfield(diffuse) - flowfield
           (jet only)).
        5. Specular mirror symmetry: the virtual-nozzle field satisfies
           T(P) = T(P') and n(P) = n(P') for P' mirrored about the plate
           line -- the defining symmetry of Fig. 6.

        Raises AssertionError on failure; returns measured deviations.
    '''
    from scipy import integrate

    results = {}
    s = np.linspace(-4.0, 4.0, 17)
    c90 = planar_surface_coefficients(s, S_0, np.pi / 2, eps, H, L)
    for key, parity in [('Cp_d', 1), ('Cq_d', 1), ('nw', 1), ('Cf_d', -1)]:
        dev = np.max(np.abs(c90[key] - parity * c90[key][::-1]))
        scale = np.max(np.abs(c90[key]))
        results[f'planar_parity_{key}'] = dev / scale
        assert dev <= 1e-12 * scale, f'2D {key} parity violated: {dev}'

    c = planar_surface_coefficients(s, S_0, alpha_0, eps, H, L)
    dev = np.max(np.abs(
        c['Cp_s'] - 2.0 * (c['Cp_d'] - eps / (2 * S_0 ** 2) * c['nw'])))
    results['planar_specular_vs_jet'] = dev
    assert dev <= 1e-14, f'2D Cp_s != 2 x jet-only Cp_d: {dev}'

    for s0 in (0.0, -2.0, 3.0):
        X, Y = planar_plate_point_coords(s0, alpha_0, L)

        def flux_integrand(theta):
            a = S_0 * np.cos(theta)
            _, A0s, _ = _scaled_planar_factors(np.asarray(a), S_0)
            return float(A0s) * max(np.sin(alpha_0 - theta), 0.0)

        I_ref, _ = integrate.quad(flux_integrand,
                                  np.arctan2(Y - H, X),
                                  np.arctan2(Y + H, X),
                                  epsabs=1e-13, epsrel=1e-12)
        nw_ref = 2.0 / np.sqrt(np.pi * eps) * I_ref
        nw_gl = float(planar_surface_coefficients(
            np.array([s0]), S_0, alpha_0, eps, H, L)['nw'][0])
        dev = abs(nw_gl - nw_ref) / nw_ref
        results[f'planar_nw_quad_s{s0}'] = dev
        assert dev <= 1e-9, f'2D n_w flux balance failed at s={s0}: {dev}'

    h = 1e-3
    n_hat = np.array([-np.sin(alpha_0), np.cos(alpha_0)])
    P = np.array([L, 0.0]) + h * n_hat
    both = planar_flowfield(P[0:1], P[1:2], S_0, alpha_0, eps, H, L, W,
                            plate='diffuse')
    jet = planar_flowfield(P[0:1], P[1:2], S_0, alpha_0, eps, H, L, W,
                           plate=None)
    nw0 = float(planar_surface_coefficients(
        np.array([0.0]), S_0, alpha_0, eps, H, L)['nw'][0])
    emission = float(both['n'][0] - jet['n'][0])
    dev = abs(emission - nw0 / 2.0) / (nw0 / 2.0)
    results['planar_halfspace_closure'] = dev
    assert dev <= 1e-2, f'2D wall-emission half-space closure: {dev}'

    pts = np.array([[1.0, 0.5], [2.5, -1.0], [3.0, 2.0], [1.5, -2.5]])
    mirrored = _mirror_about_plate_2d(pts, alpha_0, L)
    f1 = planar_flowfield(pts[:, 0], pts[:, 1], S_0, alpha_0, eps, H, L,
                          W, plate='specular')
    f2 = planar_flowfield(mirrored[:, 0], mirrored[:, 1], S_0, alpha_0,
                          eps, H, L, W, plate='specular')
    for key in ('n', 'T'):
        dev = np.max(np.abs(f1[key] - f2[key]) / np.abs(f1[key]))
        results[f'planar_specular_mirror_{key}'] = dev
        assert dev <= 1e-9, f'2D specular mirror symmetry ({key}): {dev}'

    if verbose:
        for name, value in results.items():
            print(f'  {name}: {value:.3e}')
    return results


def run_flowfield3d_sanity_checks(S_0=2.0, alpha_0=np.deg2rad(60.0),
                                  eps=1.5, R_0=0.5, L=4.0, W_0=4.0,
                                  H_0=4.0, verbose=True):
    '''
        Verification of the Section-4 flowfield-pressure implementation
        (Figs. 15-16):

        1. Solid-angle sum rule for the wall-emission kernel: a uniformly
           emitting square plate (half-side a, on-axis height h) subtends
           the closed-form solid angle 4*atan(a^2/(h*sqrt(2a^2 + h^2))),
           so the emission density must equal n_w * Omega/(4 pi) exactly
           (-> n_w/2 in the half-space limit). Validates the kernel
           constant and geometry independent of n_w.
        2. GL-vs-dblquad of the real n_w-weighted emission-density
           integral at a field point (validates the assembly with the
           Eq.-9 wall density).
        3. Specular mirror symmetry in the Y = 0 plane: n, T, p match at
           points mirrored about the plate line (the virtual-nozzle
           construction's defining property).

        Raises AssertionError on failure; returns measured deviations.
    '''
    from scipy import integrate

    results = {}
    kw = dict(S_0=S_0, alpha_0=alpha_0, eps=eps, R_0=R_0, L=L,
              W_0=W_0, H_0=H_0)

    # 1. uniform-emitter sum rule (synthetic plate, alpha_0 = 90 deg so
    # the plate normal is -x and "on-axis height" is along x)
    a_test, h_test, n_gl = 40.0, 5.0, 96
    nodes, wts = np.polynomial.legendre.leggauss(n_gl)
    Sg, Tg = np.meshgrid(a_test * nodes, a_test * nodes, indexing='ij')
    wA = np.outer(a_test * wts, a_test * wts).ravel()
    Px, Py, Pz = plate_point_coords(Sg.ravel(), Tg.ravel(), np.pi / 2, 0.0)
    dN, _, _, _, _ = _plate_emission_moments(
        np.array([-h_test]), np.array([0.0]), Px, Py, Pz,
        np.ones(Px.size), wA, np.pi / 2, eps)
    omega = 4.0 * np.arctan(a_test ** 2 / (
        h_test * np.sqrt(2 * a_test ** 2 + h_test ** 2)))
    dev = abs(float(dN[0]) - omega / (4 * np.pi)) / (omega / (4 * np.pi))
    results['3d_solid_angle_sum_rule'] = dev
    assert dev <= 1e-6, f'3D emission solid-angle sum rule: {dev}'

    # 2. GL vs dblquad with the real Eq.-9 n_w at a resolvable standoff
    n_hat2 = np.array([-np.sin(alpha_0), np.cos(alpha_0)])
    P = np.array([L, 0.0]) + 1.0 * n_hat2 + np.array([0.3, 0.0])
    both = flowfield_pressure_plane(P[0:1], P[1:2], plate='diffuse', **kw)
    jet = flowfield_pressure_plane(P[0:1], P[1:2], plate=None, **kw)
    emission_gl = float(both['n'][0] - jet['n'][0])
    n_hat3 = np.array([-np.sin(alpha_0), 0.0, np.cos(alpha_0)])

    def integrand(tau, s):
        px, py, pz = plate_point_coords(s, tau, alpha_0, L)
        nw = float(surface_coefficients(
            np.array([px]), np.array([py]), np.array([pz]),
            S_0, alpha_0, eps, R_0)['nw'][0])
        dvec = np.array([P[0] - px, -py, P[1] - pz])
        d2 = float(dvec @ dvec)
        cos_xi = max(float(dvec @ n_hat3) / np.sqrt(d2), 0.0)
        return nw * cos_xi / d2 / (4.0 * np.pi)

    emission_ref, _ = integrate.dblquad(integrand, -W_0, W_0, -H_0, H_0,
                                        epsabs=1e-10, epsrel=1e-8)
    dev = abs(emission_gl - emission_ref) / emission_ref
    results['3d_emission_gl_vs_dblquad'] = dev
    assert dev <= 1e-6, f'3D emission GL vs dblquad: {dev}'

    pts = np.array([[1.0, 0.5], [2.5, -1.0], [3.0, 2.0], [1.5, -2.5]])
    mirrored = _mirror_about_plate_2d(pts, alpha_0, L)
    f1 = flowfield_pressure_plane(pts[:, 0], pts[:, 1],
                                  plate='specular', **kw)
    f2 = flowfield_pressure_plane(mirrored[:, 0], mirrored[:, 1],
                                  plate='specular', **kw)
    for key in ('n', 'T', 'p'):
        dev = np.max(np.abs(f1[key] - f2[key]) / np.abs(f1[key]))
        results[f'3d_specular_mirror_{key}'] = dev
        assert dev <= 1e-9, f'3D specular mirror symmetry ({key}): {dev}'

    if verbose:
        for name, value in results.items():
            print(f'  {name}: {value:.3e}')
    return results


if __name__ == '__main__':
    print('Cai 2016 reference implementation sanity checks '
          '(S0=2, alpha0=60 deg, eps=1.5, L=4D):')
    run_sanity_checks()
    print('Section-3 (2D planar) checks:')
    run_planar_sanity_checks()
    print('Section-4 flowfield-pressure checks:')
    run_flowfield3d_sanity_checks()
    print('all checks passed')
    avg = averaged_coefficients(2.0, np.deg2rad(60.0), 1.5, 0.5, 4.0,
                                4.0, 4.0)
    print('Eq. 15 averaged coefficients at the paper conditions:')
    for key, value in avg.items():
        print(f'  {key} = {value:.6g}')

    # paper-figure anchors (visual validation targets)
    s = np.linspace(-5.0, 5.0, 401)
    c2d = planar_surface_coefficients(s, 2.0, np.deg2rad(60.0), 1.5,
                                      0.5, 4.0)
    print('2D anchors (S0=2, 60 deg): '
          f"peak Cp_d {c2d['Cp_d'].max():.3f} (Fig. 7 ~0.93), "
          f"peak Cp_s {c2d['Cp_s'].max():.3f} (Fig. 8 ~1.23), "
          f"peak Cf_d {c2d['Cf_d'].max():.3f} (Fig. 9 ~0.33), "
          f"peak Cq_d {c2d['Cq_d'].max():.3f} (Fig. 10 ~0.27)")
    sign_change = s[np.nonzero(np.diff(np.sign(c2d['Cf_d'])))[0]]
    print(f'  Cf_d zero crossings at s/(2H) = {np.round(sign_change, 2)} '
          '(Fig. 9 separation point ~ -2)')
    x = np.linspace(0.2, 7.0, 60)
    y = np.linspace(-3.5, 3.5, 50)
    Xg, Yg = np.meshgrid(x, y)
    Td = planar_flowfield(Xg, Yg, 2.0, np.deg2rad(60.0), 1.5, 0.5, 4.0,
                          5.0, plate='diffuse')['T']
    Ts = planar_flowfield(Xg, Yg, 2.0, np.deg2rad(60.0), 1.5, 0.5, 4.0,
                          5.0, plate='specular')['T']
    print(f'2D flowfield peaks: diffuse T/T0 {np.nanmax(Td):.2f} '
          f'(Fig. 5 top contour 2.4), specular {np.nanmax(Ts):.2f} '
          '(Fig. 6 top contour 3.5)')
    pd = flowfield_pressure_plane(Xg, Yg, 2.0, np.deg2rad(60.0), 1.5,
                                  0.5, 4.0, 4.0, 4.0, plate='diffuse')['p']
    ps = flowfield_pressure_plane(Xg, Yg, 2.0, np.deg2rad(60.0), 1.5,
                                  0.5, 4.0, 4.0, 4.0,
                                  plate='specular')['p']
    # near the exit p/p0 -> 1 for any plate; the figure anchor is the
    # impingement-center accumulation region (X > 2.5)
    imp = Xg > 2.5
    print('3D flowfield impingement-region peaks: diffuse p/p0 '
          f'{np.nanmax(pd[imp]):.2f} (Fig. 15 top contour 0.5), specular '
          f'{np.nanmax(ps[imp]):.2f} (Fig. 16 top contour 0.4)')
