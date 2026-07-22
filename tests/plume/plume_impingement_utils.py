# ========================
# PyRPOD: tests/plume/plume_impingement_utils.py
# ========================
# Shared helpers for the manual-run verification figure scripts
# plume_verification_test_28 ... _32, which reproduce Figs. 17-21 of
# Cai, C., "Gaskinetic Modeling on Dilute Gaseous Plume Impingement
# Flows," Aerospace 2016, 3(4), 43, doi:10.3390/aerospace3040043
# (Section 4: round jet impinging on an inclined rectangular plate).
#
# Design notes
# ------------
# * Paper validation conditions (Sec. 4): argon, D = 1.0 m, L = 4D,
#   plate 8 m x 8 m (W0 = H0 = L), T0 = 200 K, S0 = 2.0, Tw = 300 K
#   (eps = 1.5), alpha0 = 60 deg, fully diffuse plate. All plotted
#   quantities are the paper's dimensionless coefficients, so n_0 is
#   inert; 1e20 m^-3 matches plume_figure_utils.
# * The exact reference curves delegate to
#   pyrpod/plume/CaiImpingement2016.py (Eqs. 9-14); that module's
#   docstring records the n_w interpretation and its validation.
# * Every figure also overlays the CURRENT PYRPOD APPROXIMATION:
#   SimplifiedGasKinetics local field values (evaluated at the plume
#   coordinates distance / off-axis angle of each plate point) fed
#   through the Shen/Maxwellian wall formulas at the TRUE incidence
#   angle between the local flow direction (radial from the exit) and
#   the inclined plate normal. This replicates exactly what the
#   Phase-2-fixed strike pipeline computes per struck face, so the
#   pipeline comparison in tests/rpod is apples-to-apples with these
#   figures. The Shen shear magnitude is decomposed onto the plate's
#   (s, tau) axes along the tangential projection of the flow
#   direction to compare with the paper's Cf1/Cf2 components.
# * Specular figure (Fig. 18): the comparable Maxwellian setting is
#   sigma = 0 (fully specular reflection) -- Shen's formulas then drop
#   the wall-temperature term, matching the paper's remark that the
#   plate temperature has no effect on Cp,s.
# * Scripts _33.._40 reproduce the paper's remaining figures from the
#   same single source of truth (pyrpod/plume/CaiImpingement2016.py):
#   Figs. 5-6 (Section-3 2D flowfield temperature with a diffuse /
#   specular plate), Figs. 7-10 (2D plate surface Cp/Cf/Cq profiles),
#   and Figs. 15-16 (Section-4 flowfield pressure in the Y = 0 plane).
#   The Section-3 validation geometry (slot 2H, L = 4*(2H), plate
#   semi-width W = 5*(2H)) is read off the paper's figures -- see the
#   reference module docstring. These figures have no PyRPOD-chain
#   overlay: SimplifiedGasKinetics is a round-nozzle model (not
#   comparable to the 2D slot jet) and the strike pipeline computes no
#   flowfields-with-plates, so they are reference + digitized-DSMC
#   reproductions only.
# * Digitized-overlay slots follow tests/plume/data/digitized/README.md
#   with the cai16_ stem prefix (the 2016 paper's figure numbers would
#   otherwise collide with the 2012 slots): cai16_fig05_*.csv ...
#   cai16_fig21_*.csv (absent files silently skipped).
# * Generic figure plumbing (run_script, overlay_digitized,
#   annotate_error, max_rel_diff) is reused from plume_figure_utils;
#   only the paper-specific constants live here. Figures are saved to
#   the study's own subfolder, tests/plume/output/Cai2016, to keep the
#   Cai 2016 verification set separate from the 2012 figure outputs.

import sys
from pathlib import Path

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parents[1]
if str(_REPO_ROOT) not in sys.path:  # direct `python tests/plume/...py` runs
    sys.path.insert(0, str(_REPO_ROOT))

import plume_figure_utils as base  # noqa: E402
from plume_figure_utils import plt  # noqa: E402
from pyrpod.plume import CaiImpingement2016 as cai  # noqa: E402
from pyrpod.plume.RarefiedPlumeGasKinetics import (  # noqa: E402
    AVOGADROS_NUMBER,
    GAS_CONSTANT,
    SimplifiedGasKinetics,
    get_maxwellian_heat_transfer,
    get_maxwellian_pressure,
    get_maxwellian_shear_pressure,
)

# Cai 2016 Section-4 validation conditions (argon)
R_SPECIFIC = 208.13            # J / (kg K)
GAMMA = 5.0 / 3.0
T_0 = 200.0                    # K
N_0 = 1.0e20                   # m^-3 (inert for the coefficients)
D_NOZZLE = 1.0                 # m
R_0 = D_NOZZLE / 2.0           # m
S_0 = 2.0
U_0 = S_0 * np.sqrt(2.0 * R_SPECIFIC * T_0)   # ~577.07 m/s
T_W = 300.0                    # K
EPS = T_W / T_0                # 1.5
ALPHA_0 = np.deg2rad(60.0)
L_PLATE = 4.0 * D_NOZZLE       # m
W_0 = H_0 = L_PLATE            # plate semi-width/semi-length (8 m x 8 m)
SIGMA = 1.0

M_PARTICLE = (GAS_CONSTANT / R_SPECIFIC) / AVOGADROS_NUMBER  # kg
Q_DYN = 0.5 * N_0 * M_PARTICLE * U_0 ** 2      # n0*m*U0^2/2 (Pa)
Q_DYN_HEAT = 0.5 * N_0 * M_PARTICLE * U_0 ** 3  # n0*m*U0^3/2 (W/m^2)

GRID_N = 81                    # nodes per plate axis for contour grids

OUTPUT_DIR = base.OUTPUT_DIR / 'Cai2016'   # figure output subfolder

# Section-3 (2D slot jet) validation geometry in units of the slot
# height D2 = 2H (read off the paper's figures; see the reference
# module docstring): L = 4*(2H), plate semi-width W = 5*(2H).
D2_SLOT = 1.0
H_SLOT = D2_SLOT / 2.0
L_2D = 4.0 * D2_SLOT
W_2D = 5.0 * D2_SLOT
ALPHA_2D = np.deg2rad(60.0)    # Figs. 5-6 inclination

#: paper-legend line styles for the four-parameter profile figures
PROFILE_STYLES = ['-', '--', (0, (6, 2)), '-.']

THRUSTER_CHARACTERISTICS = {'d': D_NOZZLE, 've': U_0, 'R': R_SPECIFIC,
                            'gamma': GAMMA, 'Te': T_0, 'n': N_0}

_SIN_A, _COS_A = np.sin(ALPHA_0), np.cos(ALPHA_0)
#: plate inward normal (flow side -> plate) and in-plane axes in global coords
NORMAL_IN = np.array([_SIN_A, 0.0, -_COS_A])
TANGENT_TAU = np.array([_COS_A, 0.0, _SIN_A])
TANGENT_S = np.array([0.0, 1.0, 0.0])


def plate_axes(n=GRID_N):
    """(s, tau) axes in meters (= diameters, D = 1 m) spanning the plate."""
    return np.linspace(-W_0, W_0, n), np.linspace(-H_0, H_0, n)


def reference_grid(n=GRID_N):
    """Exact Cai 2016 coefficients on the (s, tau) tensor grid; returns
    dict of arrays shaped (len(tau), len(s)) for plt.contour."""
    s, tau = plate_axes(n)
    Sg, Tg = np.meshgrid(s, tau)
    return cai.surface_coefficients_plate(Sg, Tg, S_0, ALPHA_0, EPS, R_0,
                                          L_PLATE)


def chain_point_loads(X, Y, Z, sigma=SIGMA):
    """Current-PyRPOD approximation at one global plate point (X, Y, Z):
    SimplifiedGasKinetics field state at the point's plume coordinates,
    Shen/Maxwellian wall loads at the true incidence angle.

    Returns (pressure, shear_magnitude, heat_flux) in SI units. This is
    the per-face computation of the Phase-2-fixed strike pipeline
    (PlumeStrikeCalculator), kept in one helper so the figures and the
    pipeline comparison test share a single definition.
    """
    dist = float(np.sqrt(X * X + Y * Y + Z * Z))
    theta_pos = float(np.arctan2(np.hypot(Y, Z), X))
    plume = SimplifiedGasKinetics(dist, theta_pos, THRUSTER_CHARACTERISTICS,
                                  T_W, sigma)
    if theta_pos != 0.0:
        n_ratio = plume.get_num_density_ratio()
        T = T_0 * plume.get_temp_ratio()
        u = plume.get_U_normalized() / plume.beta_0
        w = plume.get_W_normalized() / plume.beta_0
        U = float(np.hypot(u, w))
    else:
        n_ratio = plume.get_num_density_centerline()
        T = T_0 * plume.get_temp_centerline()
        U = plume.get_velocity_centerline() / plume.beta_0
    rho_inf = N_0 * n_ratio * M_PARTICLE
    S = U * plume.get_beta(T)

    cos_inc = (X * _SIN_A - Z * _COS_A) / dist   # flow dir . inward normal
    incidence = float(np.arccos(np.clip(cos_inc, -1.0, 1.0)))

    pressure = get_maxwellian_pressure(rho_inf, U, S, sigma, incidence,
                                       T, T_W)
    shear = abs(get_maxwellian_shear_pressure(rho_inf, U, S, sigma,
                                              incidence))
    heat = get_maxwellian_heat_transfer(rho_inf, S, sigma, incidence,
                                        T, T_W, R_SPECIFIC, GAMMA)
    return pressure, shear, heat


def chain_grid(n=GRID_N, sigma=SIGMA):
    """Current-PyRPOD approximation coefficients on the (s, tau) grid.

    Returns dict with 'Cp', 'Cf1', 'Cf2', 'Cshear', 'Cq' shaped like
    reference_grid arrays. The shear magnitude from Shen's formula acts
    along the tangential projection of the (radial) flow direction; its
    components on the plate axes give Cf1 (inclined direction) and Cf2
    (horizontal), signed like the paper's Figs. 19-20.
    """
    s, tau = plate_axes(n)
    Sg, Tg = np.meshgrid(s, tau)
    X, Y, Z = cai.plate_point_coords(Sg, Tg, ALPHA_0, L_PLATE)

    Cp = np.empty_like(X)
    Cf1 = np.empty_like(X)
    Cf2 = np.empty_like(X)
    Csh = np.empty_like(X)
    Cq = np.empty_like(X)
    it = np.nditer(X, flags=['multi_index'])
    for _ in it:
        i = it.multi_index
        x, y, z = float(X[i]), float(Y[i]), float(Z[i])
        p, sh, q = chain_point_loads(x, y, z, sigma=sigma)
        dist = np.sqrt(x * x + y * y + z * z)
        u_hat = np.array([x, y, z]) / dist
        cos_inc = float(u_hat @ NORMAL_IN)
        t_vec = u_hat - cos_inc * NORMAL_IN
        t_norm = np.linalg.norm(t_vec)
        t_hat = t_vec / t_norm if t_norm > 1e-12 else np.zeros(3)
        Cp[i] = p / Q_DYN
        Csh[i] = sh / Q_DYN
        Cf1[i] = sh * float(t_hat @ TANGENT_TAU) / Q_DYN
        Cf2[i] = sh * float(t_hat @ TANGENT_S) / Q_DYN
        Cq[i] = q / Q_DYN_HEAT
    return {'Cp': Cp, 'Cf1': Cf1, 'Cf2': Cf2, 'Cshear': Csh, 'Cq': Cq}


def save_figure(fig, name):
    """Save PNG to tests/plume/output/Cai2016 and return the path."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_DIR / f'{name}.png'
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    return path


def mean_rel_diff(candidate, reference, mask):
    """mean |candidate - reference| / |reference| over the masked entries."""
    candidate = np.asarray(candidate, dtype=float)
    reference = np.asarray(reference, dtype=float)
    m = mask & np.isfinite(candidate) & np.isfinite(reference)
    return float(np.mean(np.abs(candidate[m] - reference[m])
                         / np.abs(reference[m])))


def significant_mask(reference, fraction=0.05):
    """Entries where |reference| >= fraction * max|reference| -- the
    region where relative errors are meaningful (the largest relative
    differences otherwise sit in the near-zero plate corners, the same
    caveat the 2012 error summary records)."""
    reference = np.asarray(reference, dtype=float)
    return np.abs(reference) >= fraction * np.max(np.abs(reference))


def impingement_contour_figure(fig_stem, ref_field, chain_field, levels,
                               title, file_name, chain_label='PyRPOD chain',
                               fmt='%g'):
    """Shared Figs. 17-21 builder: reference contours (solid black),
    current-PyRPOD-approximation contours (dashed red, same levels),
    digitized slots, and a max/mean relative-error annotation over the
    significant region."""
    s, tau = plate_axes()

    fig, ax = plt.subplots(figsize=(6, 6))
    cs = ax.contour(s, tau, ref_field, levels=levels, colors='k',
                    linewidths=1.2)
    ax.clabel(cs, fmt=fmt, fontsize=7)
    ax.contour(s, tau, chain_field, levels=levels, colors='r',
               linewidths=0.9, linestyles='dashed')
    n_dig = base.overlay_digitized(ax, fig_stem, style='line')

    ax.set_xlim(-W_0, W_0)
    ax.set_ylim(-H_0, H_0)
    ax.set_xlabel('s')
    ax.set_ylabel(r'$\tau$')
    ax.set_title(title)
    handles = [plt.Line2D([], [], color='k', lw=1.2,
                          label='Cai 2016 (exact)'),
               plt.Line2D([], [], color='r', lw=0.9, ls='--',
                          label=chain_label)]
    if n_dig:
        handles.append(plt.Line2D([], [], color='k', lw=1.0,
                                  label='DSMC (digitized)'))
    ax.legend(handles=handles, fontsize=7, loc='upper right')

    mask = significant_mask(ref_field)
    base.annotate_error(
        ax,
        f'max rel diff = {base.max_rel_diff(chain_field[mask], ref_field[mask]):.2g}\n'
        f'mean rel diff = {mean_rel_diff(chain_field, ref_field, mask):.2g}\n'
        r'(where $|C_{ref}| \geq$ 5% of peak)')
    return save_figure(fig, file_name)


def _draw_plate_trace(ax, alpha_0, L, semi_length):
    """Plate trace in the plotted plane (2D plate line or the 3D plate's
    Y = 0 section), drawn like the paper's contour figures."""
    tau = np.array([-semi_length, semi_length])
    ax.plot(L + tau * np.cos(alpha_0), tau * np.sin(alpha_0), 'k-', lw=1.6)


def planar_temperature_contour_figure(plate, levels, fig_stem, file_name,
                                      title, S_0=2.0, grid_n=161):
    """Figs. 5-6 builder: 2D flowfield temperature contours T/T0 for the
    slot jet with a diffuse or specular plate (delegates to
    cai.planar_flowfield), solid black with labels, plate line drawn,
    digitized slots overlaid."""
    x = np.linspace(0.05, 8.0, grid_n)
    y = np.linspace(-4.0, 4.0, grid_n)
    Xg, Yg = np.meshgrid(x * D2_SLOT, y * D2_SLOT)
    T = cai.planar_flowfield(Xg, Yg, S_0, ALPHA_2D, EPS, H_SLOT, L_2D,
                             W_2D, plate=plate)['T']

    fig, ax = plt.subplots(figsize=(6, 6))
    cs = ax.contour(x, y, T, levels=levels, colors='k', linewidths=1.1)
    ax.clabel(cs, fmt='%g', fontsize=7)
    _draw_plate_trace(ax, ALPHA_2D, L_2D, W_2D)
    n_dig = base.overlay_digitized(ax, fig_stem, style='line')

    ax.set_xlim(0, 8)
    ax.set_ylim(-4, 4)
    ax.set_xlabel('X/(2H)')
    ax.set_ylabel('Y/(2H)')
    ax.set_title(title)
    handles = [plt.Line2D([], [], color='k', lw=1.1, label='Analytical')]
    if n_dig:
        handles.append(plt.Line2D([], [], color='k', lw=1.0,
                                  label='DSMC (digitized)'))
    ax.legend(handles=handles, fontsize=7, loc='lower right')
    return save_figure(fig, file_name)


def planar_profile_figure(quantity, params, fig_stem, file_name, title,
                          ylabel, ylim, n_s=401):
    """Figs. 7-10 builder: 2D plate surface coefficient profiles vs
    s/(2H) for (S_0, alpha_0 deg) parameter combinations (delegates to
    cai.planar_surface_coefficients), paper-style black line styles,
    digitized slots overlaid."""
    s = np.linspace(-5.0, 5.0, n_s) * D2_SLOT

    fig, ax = plt.subplots(figsize=(6, 4.5))
    for (S_0, alpha_deg), ls in zip(params, PROFILE_STYLES):
        coeffs = cai.planar_surface_coefficients(
            s, S_0, np.deg2rad(alpha_deg), EPS, H_SLOT, L_2D)
        ax.plot(s / D2_SLOT, coeffs[quantity], color='k', ls=ls, lw=1.1,
                label=f'$S_0$={S_0:g}, {alpha_deg:g}$^\\circ$')
    base.overlay_digitized(ax, fig_stem)

    ax.set_xlim(-5, 6)
    ax.set_ylim(*ylim)
    ax.set_xlabel('s/(2H)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=8)
    return save_figure(fig, file_name)


def pressure_contour_figure_3d(plate, levels, fig_stem, file_name, title,
                               S_0=2.0, grid_n=141):
    """Figs. 15-16 builder: Section-4 flowfield static-pressure contours
    p/p0 in the vertical Y = 0 plane with a diffuse or specular plate
    (delegates to cai.flowfield_pressure_plane), plate section drawn,
    digitized slots overlaid."""
    x = np.linspace(0.05, 8.0, grid_n)
    z = np.linspace(-4.0, 4.0, grid_n)
    Xg, Zg = np.meshgrid(x * D_NOZZLE, z * D_NOZZLE)
    p = cai.flowfield_pressure_plane(
        Xg, Zg, S_0, ALPHA_0, EPS, R_0, L_PLATE, W_0, H_0,
        plate=plate)['p']

    fig, ax = plt.subplots(figsize=(6, 6))
    cs = ax.contour(x, z, p, levels=levels, colors='k', linewidths=1.1)
    ax.clabel(cs, fmt='%g', fontsize=7)
    _draw_plate_trace(ax, ALPHA_0, L_PLATE, H_0)
    n_dig = base.overlay_digitized(ax, fig_stem, style='line')

    ax.set_xlim(0, 8)
    ax.set_ylim(-4, 4)
    ax.set_xlabel('$X/(2R_0)$')
    ax.set_ylabel('$W/(2R_0)$')
    ax.set_title(title)
    handles = [plt.Line2D([], [], color='k', lw=1.1, label='Analytical')]
    if n_dig:
        handles.append(plt.Line2D([], [], color='k', lw=1.0,
                                  label='DSMC (digitized)'))
    ax.legend(handles=handles, fontsize=7, loc='lower right')
    return save_figure(fig, file_name)
