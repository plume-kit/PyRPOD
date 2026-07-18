# ========================
# PyRPOD: tests/plume/plume_figure_utils.py
# ========================
# Shared helpers for the manual-run verification figure scripts
# plume_verification_test_04 ... _27, which reproduce Figs. 2-25 of
# Cai, C. and Wang, L., "Numerical Validations for a Set of Collisionless
# Rocket Plume Solutions," JSR 49(1), 2012, DOI 10.2514/1.A32046.
#
# Design notes
# ------------
# * All figures use the paper's validation conditions: argon,
#   D = 0.2 m, gamma = 5/3, R = 208.13 J/(kg K). Every plotted quantity
#   is normalized (n/n0, U*sqrt(beta0), T/T0, ...), so the absolute
#   T_0 and n_0 values are inert; the ones below match the existing
#   verification tests. ve is set from the exit speed ratio,
#   ve = S_0 * sqrt(2*R*T_0).
# * Full-model contour fields are evaluated with a vectorized
#   Gauss-Legendre quadrature (fixed order 80 per axis, the same order
#   at which CollisionlessGasKinetics converges to 1e-9). Every call
#   cross-checks a few sample points against the class and raises if
#   they disagree beyond 1e-6, so the vectorized path cannot silently
#   diverge from the physics module.
# * The models require X > 0; grids start at X_MIN_OVER_D and angular
#   sweeps stop at THETA_MAX_DEG < 90 deg.
# * Digitized-data convention (see tests/plume/data/digitized/README.md):
#   any CSV named '<figstem>_<series>.csv' (two columns x,y, one header
#   row) is overlaid automatically by overlay_digitized(); absent files
#   are silently skipped, so the DSMC series appear once the paper's
#   curves are digitized.
# * Simons overlays are EXIT-referenced (n/n_0) via the isentropic
#   throat-to-exit conversion with Me = S_0*sqrt(2/gamma) (from
#   S_0 = U0/sqrt(2RT0) and M = U0/sqrt(gamma*R*T0)). Following the
#   paper's remark that the cosine-law curves with different kappa
#   coincide on the centerline (p. 65: "the cosine-law model with
#   different kappa values actually provides the same value"), the
#   normalization constant A is always taken from the Boyton default
#   kappa while the plotted decay exponent may vary (Figs. 22-24).

import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')  # save-only figures; must precede pyplot import
import matplotlib.pyplot as plt  # noqa: E402  (backend must be set first)

_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parents[1]
if str(_REPO_ROOT) not in sys.path:  # direct `python tests/plume/...py` runs
    sys.path.insert(0, str(_REPO_ROOT))

from pyrpod.plume.RarefiedPlumeGasKinetics import (  # noqa: E402
    CollisionlessGasKinetics,
    SimplifiedGasKinetics,
    Simons,
    get_K_factor,
    get_M_factor,
    get_N_factor,
    get_Q_full,
)

# Paper validation conditions (Sec. III: argon, D = 0.2 m)
R_SPECIFIC = 208.13    # J / (kg K)
T_0 = 500.0            # K (inert for normalized results)
GAMMA = 5.0 / 3.0
N_0 = 1.0e20           # m^-3 (inert for normalized results)
D_NOZZLE = 0.2         # m
R_0 = D_NOZZLE / 2     # m
T_W = 300.0            # K (unused by plotted quantities)
SIGMA = 1.0
BOLTZMANN = 1.380649e-23  # J / K

OUTPUT_DIR = _THIS_DIR / 'output'
DIGITIZED_DIR = _THIS_DIR / 'data' / 'digitized'

X_MIN_OVER_D = 0.05    # models require X > 0
THETA_MAX_DEG = 89.5   # angular sweeps stop short of 90 deg (X -> 0)
QUAD_ORDER = 80        # per-axis Gauss-Legendre order for grid fields
GRID_N = 161           # nodes along the longer contour-grid axis

_GL_NODES, _GL_WEIGHTS = np.polynomial.legendre.leggauss(QUAD_ORDER)


# ---------------------------------------------------------------------------
# model factories
# ---------------------------------------------------------------------------

def thruster_characteristics(S_0):
    """Thruster dict for the gas-kinetic classes at exit speed ratio S_0."""
    ve = S_0 * np.sqrt(2 * R_SPECIFIC * T_0)
    return {'d': D_NOZZLE, 've': ve, 'R': R_SPECIFIC,
            'gamma': GAMMA, 'Te': T_0, 'n': N_0}


def make_plume(cls, distance, theta, S_0):
    """Instantiate SimplifiedGasKinetics or CollisionlessGasKinetics."""
    return cls(distance, theta, thruster_characteristics(S_0), T_W, SIGMA)


def exit_mach(S_0):
    """Me from S_0: S_0 = U0/sqrt(2RT0), M = U0/sqrt(gamma R T0)."""
    return S_0 * np.sqrt(2.0 / GAMMA)


def throat_to_exit_density_ratio(S_0):
    """n_s/n_0 used by Simons exit referencing [module docstring note]."""
    Me = exit_mach(S_0)
    return ((1 + (GAMMA - 1) / 2 * Me ** 2)
            / ((GAMMA + 1) / 2)) ** (1 / (GAMMA - 1))


def make_simons(distance, S_0, kappa=None):
    """Simons instance with chamber conditions isentropically consistent
    with the exit state at Me(S_0). T_c/P_c only rescale absolute
    density; they are inert for the plotted ratios."""
    Me = exit_mach(S_0)
    stag = 1 + (GAMMA - 1) / 2 * Me ** 2
    T_c = T_0 * stag
    P_c = (N_0 * BOLTZMANN * T_0) * stag ** (GAMMA / (GAMMA - 1))
    return Simons(GAMMA, R_SPECIFIC, T_c, P_c, R_0, distance, kappa=kappa)


def simons_density_field(X, Z, S_0, kappa_f=None):
    """Exit-referenced Simons density n/n_0 at points (X, 0, Z) [m].

    A is always the Boyton-kappa normalization constant (paper p. 65:
    the cosine-law curves coincide at theta = 0 for all kappa); kappa_f
    optionally varies the plotted decay exponent f = cos^kappa_f
    (Figs. 22-24). Vectorized; theta >= theta_max gives 0.
    """
    X = np.asarray(X, dtype=float)
    Z = np.asarray(Z, dtype=float)
    boyton = make_simons(1.0, S_0)          # A from default Boyton kappa
    theta_max = boyton.get_limiting_turn_angle()
    kappa = boyton.kappa if kappa_f is None else kappa_f

    r_sph = np.sqrt(X ** 2 + Z ** 2)
    theta = np.arctan2(np.abs(Z), X)
    f = np.where(theta < theta_max,
                 np.cos((np.pi / 2) * (np.minimum(theta, theta_max)
                                       / theta_max)) ** kappa,
                 0.0)
    n_over_ns = boyton.A * (R_0 / r_sph) ** 2 * f
    return n_over_ns * throat_to_exit_density_ratio(S_0)


# ---------------------------------------------------------------------------
# centerline profiles (exact closed forms, Eqs. 18/19/21)
# ---------------------------------------------------------------------------

def centerline_profiles(x_over_D, S_0, quantities=('n',)):
    """Exact analytical centerline values at X/D array; quantities from
    {'n', 'U', 'T'} -> Eqs. 18 / 19 / 21."""
    getters = {'n': 'get_num_density_centerline',
               'U': 'get_velocity_centerline',
               'T': 'get_temp_centerline'}
    out = {q: np.empty(len(x_over_D)) for q in quantities}
    for i, xd in enumerate(x_over_D):
        plume = make_plume(SimplifiedGasKinetics, xd * D_NOZZLE, 0.0, S_0)
        for q in quantities:
            out[q][i] = getattr(plume, getters[q])()
    return out


def simplified_centerline_density(x_over_D, S_0):
    """Simplified-model centerline density, Eq. 14 with Q' = 1."""
    X = np.asarray(x_over_D) * D_NOZZLE
    return get_K_factor(1.0, S_0) / (2 * np.sqrt(np.pi)) * (R_0 / X) ** 2


# ---------------------------------------------------------------------------
# vectorized full-model field evaluation (Eqs. 5-8)
# ---------------------------------------------------------------------------

def full_field_values(quantity, X, Z, S_0, _chunk=256):
    """Full-model field quantity at 1-D point arrays X, Z [m].

    quantity: 'n', 'U', 'W', 'Vr', 'T', or 'p'. Same integrals as
    CollisionlessGasKinetics at its converged order (80); each call
    verifies sample points against the class to 1e-6 (see module
    docstring).
    """
    X = np.asarray(X, dtype=float).ravel()
    Z = np.asarray(Z, dtype=float).ravel()

    r = 0.5 * R_0 * (_GL_NODES + 1)
    w_r = 0.5 * R_0 * _GL_WEIGHTS
    eps = 0.5 * np.pi * _GL_NODES
    w_eps = 0.5 * np.pi * _GL_WEIGHTS
    Rn, En = np.meshgrid(r, eps, indexing='ij')
    W2D = np.outer(w_r, w_eps)
    sinE = np.sin(En)

    n_pts = X.size
    I_K = np.empty(n_pts)
    I_M = np.empty(n_pts)
    I_W = np.empty(n_pts)
    I_N = np.empty(n_pts)
    for start in range(0, n_pts, _chunk):
        sl = slice(start, min(start + _chunk, n_pts))
        Xc = X[sl][:, None, None]
        Zc = Z[sl][:, None, None]
        Q = Xc ** 2 / (Xc ** 2 + Zc ** 2 - 2 * Zc * Rn * sinE + Rn ** 2)
        K = get_K_factor(Q, S_0)
        M = get_M_factor(Q, S_0)
        N = get_N_factor(Q, S_0)
        I_K[sl] = np.sum(W2D * Rn * K, axis=(1, 2))
        I_M[sl] = np.sum(W2D * Rn * M, axis=(1, 2))
        I_W[sl] = np.sum(W2D * (Zc - Rn * sinE) * Rn * M, axis=(1, 2))
        I_N[sl] = np.sum(W2D * Rn * N, axis=(1, 2))

    n = I_K / (np.pi ** 1.5 * X ** 2)
    U = I_M / I_K
    W = I_W / (X * I_K)
    T = -(2 / 3) * (U ** 2 + W ** 2) + (4 / 3) * I_N / I_K
    values = {'n': n, 'U': U, 'W': W, 'T': T,
              'Vr': (X * U + Z * W) / np.sqrt(X ** 2 + Z ** 2),
              'p': n * T}
    _verify_against_class(values, X, Z, S_0)
    return values[quantity]


def _verify_against_class(values, X, Z, S_0, rtol=1e-6):
    """Cross-check sample points against CollisionlessGasKinetics."""
    for idx in {0, X.size // 2, X.size - 1}:
        d = float(np.hypot(X[idx], Z[idx]))
        th = float(np.arctan2(Z[idx], X[idx]))
        ref = make_plume(CollisionlessGasKinetics, d, th, S_0)
        checks = {'n': ref.get_num_density_ratio(),
                  'U': ref.get_U_normalized(),
                  'T': ref.get_temp_ratio()}
        for q, expected in checks.items():
            got = values[q][idx]
            if abs(got - expected) > rtol * abs(expected):
                raise AssertionError(
                    f'vectorized field {q} diverged from '
                    f'CollisionlessGasKinetics at (X={X[idx]}, Z={Z[idx]}): '
                    f'{got} vs {expected}')


def full_field_grid(quantity, x_over_D, z_over_D, S_0):
    """Full-model quantity on the tensor grid; returns shape
    (len(z_over_D), len(x_over_D)) for use with plt.contour."""
    Xg, Zg = np.meshgrid(np.asarray(x_over_D) * D_NOZZLE,
                         np.asarray(z_over_D) * D_NOZZLE)
    vals = full_field_values(quantity, Xg.ravel(), Zg.ravel(), S_0)
    return vals.reshape(Xg.shape)


def simplified_field_grid(quantity, x_over_D, z_over_D, S_0):
    """Simplified-model quantity (Eqs. 13-17) on the tensor grid."""
    Xg, Zg = np.meshgrid(np.asarray(x_over_D) * D_NOZZLE,
                         np.asarray(z_over_D) * D_NOZZLE)
    Q = Xg ** 2 / (Xg ** 2 + Zg ** 2)
    K = get_K_factor(Q, S_0)
    M = get_M_factor(Q, S_0)
    N = get_N_factor(Q, S_0)
    n = K / (2 * np.sqrt(np.pi)) * (R_0 / Xg) ** 2
    U = M / K
    W = U * Zg / Xg
    T = -2 * M ** 2 / (3 * Q * K ** 2) + 4 * N / (3 * K)
    values = {'n': n, 'U': U, 'W': W, 'T': T,
              'Vr': (Xg * U + Zg * W) / np.sqrt(Xg ** 2 + Zg ** 2),
              'p': n * T}
    return values[quantity]


def default_contour_axes(x_max=10.0, z_max=10.0):
    """Paper-style contour grid: X/D in [X_MIN, x_max], Z/D in [0, z_max]
    (fields are symmetric in Z; mirror for the lower half-plane)."""
    x = np.linspace(X_MIN_OVER_D, x_max, GRID_N)
    z = np.linspace(0.0, z_max, (GRID_N + 1) // 2)
    return x, z


# ---------------------------------------------------------------------------
# digitized-data overlay (see data/digitized/README.md)
# ---------------------------------------------------------------------------

def load_digitized(fig_stem):
    """dict {series_suffix: (x, y)} from data/digitized/<figstem>_*.csv."""
    datasets = {}
    for path in sorted(DIGITIZED_DIR.glob(f'{fig_stem}_*.csv')):
        data = np.genfromtxt(path, delimiter=',', skip_header=1)
        data = np.atleast_2d(data)
        datasets[path.stem[len(fig_stem) + 1:]] = (data[:, 0], data[:, 1])
    return datasets


def _pretty_series_label(suffix):
    tokens = []
    for token in suffix.split('_'):
        if token.lower() == 'dsmc':
            tokens.append('DSMC')
        elif token.lower().startswith('kn'):
            tokens.append('Kn=' + token[2:].replace('p', '.'))
        else:
            tokens.append(token)
    return ' '.join(tokens)


def overlay_digitized(ax, fig_stem, style='markers', mirror_z=False, **kw):
    """Overlay every digitized series for fig_stem; returns #series.

    style 'markers' suits profile figures; 'line' suits digitized
    contour polylines. mirror_z flips y (for lower-half-plane DSMC
    slots in the split contour figures).
    """
    count = 0
    for suffix, (x, y) in load_digitized(fig_stem).items():
        y = -y if mirror_z else y
        label = _pretty_series_label(suffix)
        if style == 'line':
            ax.plot(x, y, color='k', lw=1.0, label=label, **kw)
        else:
            ax.plot(x, y, linestyle='none', marker='d', mfc='none',
                    color='k', ms=4, label=label, **kw)
        count += 1
    return count


# ---------------------------------------------------------------------------
# figure assembly helpers
# ---------------------------------------------------------------------------

def max_rel_diff(candidate, reference):
    """max |candidate - reference| / |reference| over finite entries."""
    candidate = np.asarray(candidate, dtype=float)
    reference = np.asarray(reference, dtype=float)
    mask = np.isfinite(candidate) & np.isfinite(reference) & (reference != 0)
    return float(np.max(np.abs(candidate[mask] - reference[mask])
                        / np.abs(reference[mask])))


def annotate_error(ax, text, loc='lower left'):
    """Small model-difference annotation box on the axes."""
    positions = {'lower left': (0.02, 0.02, 'left', 'bottom'),
                 'lower right': (0.98, 0.02, 'right', 'bottom'),
                 'upper left': (0.02, 0.98, 'left', 'top')}
    x, y, ha, va = positions[loc]
    ax.text(x, y, text, transform=ax.transAxes, fontsize=7, ha=ha, va=va,
            bbox=dict(boxstyle='round', fc='white', ec='0.6', alpha=0.85))


def save_figure(fig, name):
    """Save PNG to tests/plume/output and return the path."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_DIR / f'{name}.png'
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    return path


def run_script(generate_figure):
    """Manual-run entry point shared by all figure scripts."""
    t0 = time.perf_counter()
    path = generate_figure()
    print(f'saved {path}  ({time.perf_counter() - t0:.1f} s)')


# ---------------------------------------------------------------------------
# shared builders for the Kn-triplet figures (identical analytic content;
# separate figures so per-Kn DSMC overlays land in the right file)
# ---------------------------------------------------------------------------

def density_contour_figure(kn_label, fig_stem):
    """Figs. 6-8: number-density contours, S_0 = 2. Upper half-plane:
    analytical (solid) + simplified (dashed); lower half-plane: Simons
    (dashed) + DSMC digitized slot (solid lines when data present)."""
    S_0 = 2.0
    levels = [0.001, 0.01, 0.1]
    x, z = default_contour_axes()
    n_full = full_field_grid('n', x, z, S_0)
    n_simp = simplified_field_grid('n', x, z, S_0)
    Xg, Zg = np.meshgrid(x * D_NOZZLE, z * D_NOZZLE)
    n_simons = simons_density_field(Xg, Zg, S_0)

    fig, ax = plt.subplots(figsize=(6, 7))
    cs = ax.contour(x, z, n_full, levels=levels, colors='k',
                    linewidths=1.2)
    ax.clabel(cs, fmt='%g', fontsize=7)
    cs = ax.contour(x, z, n_simp, levels=levels, colors='k',
                    linewidths=0.9, linestyles='dashed')
    ax.clabel(cs, fmt='%g', fontsize=7)
    cs = ax.contour(x, -z, n_simons, levels=levels, colors='k',
                    linewidths=0.9, linestyles='dashed')
    ax.clabel(cs, fmt='%g', fontsize=7)
    n_dig = overlay_digitized(ax, fig_stem, style='line', mirror_z=False)

    ax.axhline(0.0, color='k', lw=0.6)
    ax.set_xlim(0, 10)
    ax.set_ylim(-10, 10)
    ax.set_xlabel('X/D')
    ax.set_ylabel('Z/D')
    ax.set_title(f'Normalized number density, Kn = {kn_label}, '
                 f'$S_0$ = 2.0')
    handles = [plt.Line2D([], [], color='k', lw=1.2, label='Analytical (top)'),
               plt.Line2D([], [], color='k', lw=0.9, ls='--',
                          label='Simplified (top)'),
               plt.Line2D([], [], color='k', lw=0.9, ls='--',
                          label='Simons (bottom)')]
    if n_dig:
        handles.append(plt.Line2D([], [], color='k', lw=1.0,
                                  label='DSMC (bottom, digitized)'))
    ax.legend(handles=handles, fontsize=7, loc='upper right')
    far = x >= 1.0  # near the exit the simplified (R_0/X)^2 form diverges
    annotate_error(
        ax,
        f'max |n_S/n_A - 1| = {max_rel_diff(n_simp[:, far], n_full[:, far]):.2g}\n'
        f'max |n_Sim/n_A - 1| = {max_rel_diff(n_simons[:, far], n_full[:, far]):.2g}\n'
        '(over grid with X/D $\\geq$ 1; largest in the plume tails)')
    return save_figure(fig, fig_stem)


def top_bottom_contour_figure(quantity, levels, kn_label, fig_stem,
                              title_quantity, x_max=10.0, z_max=10.0,
                              z_axis_label='Z/D', fmt='%g'):
    """Figs. 11-18 pattern: analytical contours in the upper half-plane,
    lower half-plane reserved for the DSMC digitized overlay."""
    S_0 = 2.0
    x, z = default_contour_axes(x_max, z_max)
    field = full_field_grid(quantity, x, z, S_0)

    fig, ax = plt.subplots(figsize=(6, 7))
    cs = ax.contour(x, z, field, levels=levels, colors='k', linewidths=1.1)
    ax.clabel(cs, fmt=fmt, fontsize=7)
    n_dig = overlay_digitized(ax, fig_stem, style='line')

    ax.axhline(0.0, color='k', lw=0.6)
    ax.set_xlim(0, x_max)
    ax.set_ylim(-z_max, z_max)
    ax.set_xlabel('X/D')
    ax.set_ylabel(z_axis_label)
    ax.set_title(f'{title_quantity}, Kn = {kn_label}, $S_0$ = 2.0')
    ax.text(0.7 * x_max, 0.55 * z_max, 'Analytical', fontsize=9)
    ax.text(0.7 * x_max, -0.55 * z_max,
            'DSMC' if n_dig else 'DSMC (pending digitized data)',
            fontsize=9)
    return save_figure(fig, fig_stem)


def centerline_density_profile_figure(kn_label, fig_stem):
    """Figs. 19-21: centerline density -- analytical (Eq. 18, circles),
    simplified (Eq. 14, triangles), Simons (solid line, exit-referenced),
    DSMC digitized slot."""
    S_0 = 2.0
    x_over_D = np.linspace(X_MIN_OVER_D, 10.0, 200)
    analytical = centerline_profiles(x_over_D, S_0, ('n',))['n']
    simplified = simplified_centerline_density(x_over_D, S_0)
    r = x_over_D * D_NOZZLE
    simons = simons_density_field(r, np.zeros_like(r), S_0)

    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.plot(x_over_D, simons, 'k-', lw=1.2, label='Simons')
    ax.plot(x_over_D, analytical, linestyle='none', marker='o', mfc='none',
            color='k', ms=4, markevery=4, label='Analytical')
    ax.plot(x_over_D, simplified, linestyle='none', marker='>', color='k',
            ms=4, markevery=(2, 4), label='Simplified')
    overlay_digitized(ax, fig_stem)

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 1.2)
    ax.set_xlabel('X/D')
    ax.set_ylabel('$n/n_0$')
    ax.set_title(f'Centerline density profiles, Kn = {kn_label}, '
                 f'$S_0$ = 2.0')
    ax.legend(fontsize=8)
    tail = x_over_D >= 1.0
    annotate_error(
        ax,
        f'max |n_S/n_A - 1| = {max_rel_diff(simplified[tail], analytical[tail]):.2g} '
        f'(X/D $\\geq$ 1)\n'
        f'max |n_Sim/n_A - 1| = {max_rel_diff(simons[tail], analytical[tail]):.2g} '
        f'(X/D $\\geq$ 1)', loc='lower right')
    return save_figure(fig, fig_stem)


def angular_density_profile_figure(kn_label, fig_stem):
    """Figs. 22-24: density along r/D = 10 -- analytical, simplified,
    Simons kappa = 1.5/2/3 (single Boyton-A normalization, see module
    docstring), DSMC digitized slot."""
    S_0 = 2.0
    r = 10.0 * D_NOZZLE
    theta = np.deg2rad(np.linspace(0.0, THETA_MAX_DEG, 90))
    X = r * np.cos(theta)
    Z = r * np.sin(theta)
    analytical = full_field_values('n', X, Z, S_0)
    Q = X ** 2 / (X ** 2 + Z ** 2)
    simplified = (get_K_factor(Q, S_0) / (2 * np.sqrt(np.pi))
                  * (R_0 / X) ** 2)
    theta_deg = np.rad2deg(theta)

    fig, ax = plt.subplots(figsize=(6, 4.5))
    for kappa, ls in [(2.0, '-'), (1.5, '--'), (3.0, '-.')]:
        simons = simons_density_field(X, Z, S_0, kappa_f=kappa)
        ax.plot(theta_deg, simons, 'k', ls=ls, lw=1.1,
                label=f'Simons $\\kappa$={kappa:g}')
    ax.plot(theta_deg, analytical, linestyle='none', marker='o', mfc='none',
            color='k', ms=4, markevery=2, label='Analytical')
    ax.plot(theta_deg, simplified, linestyle='none', marker='>', color='k',
            ms=4, markevery=(1, 2), label='Simplified')
    overlay_digitized(ax, fig_stem)

    ax.set_xlim(0, 90)
    ax.set_ylim(0, 0.015)
    ax.set_xticks([0, 30, 60, 90])
    ax.set_xlabel(r'$\theta$')
    ax.set_ylabel('$n/n_0$')
    ax.set_title(f'Density profiles along r/D = 10, Kn = {kn_label}, '
                 f'$S_0$ = 2.0')
    ax.legend(fontsize=8)
    simons_boyton = simons_density_field(X, Z, S_0)
    annotate_error(
        ax,
        f'max |n_S/n_A - 1| = {max_rel_diff(simplified, analytical):.2g}\n'
        f'max |n_Sim($\\kappa$=3)/n_A - 1| = '
        f'{max_rel_diff(simons_boyton, analytical):.2g}',
        loc='lower left')
    return save_figure(fig, fig_stem)
