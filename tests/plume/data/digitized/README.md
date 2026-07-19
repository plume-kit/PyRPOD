# Digitized reference data for the plume verification figures

Drop digitized curves from Cai & Wang 2012 (JSR 49(1), DOI
10.2514/1.A32046) here and the manual-run figure scripts
(`tests/plume/plume_verification_test_04` ... `_27`) will overlay them
automatically on their next run. No code changes are needed.

The same convention serves the Cai 2016 impingement figures
(Aerospace 3(4):43, DOI 10.3390/aerospace3040043) reproduced by
`plume_verification_test_28` ... `_40`, with a `cai16_` stem prefix so
the 2016 paper's figure numbers never collide with the 2012 slots
above: `cai16_fig05_*.csv` ... `cai16_fig10_*.csv` (Section 3, 2D
planar plate), `cai16_fig15_*.csv`, `cai16_fig16_*.csv` (flowfield
pressure contours), and `cai16_fig17_*.csv` ... `cai16_fig21_*.csv`
(3D plate surface coefficients). One CSV per digitized DSMC/analytic
curve or contour polyline in the figure's plotted units, e.g.
`cai16_fig17_dsmc_0p2.csv` for the Cp = 0.2 line of the 2016 Fig. 17,
or `cai16_fig07_dsmc.csv` for the DSMC Cp profile of the 2016 Fig. 7.

## File convention

- Name: `<figstem>_<series>.csv`, where `<figstem>` is the paper figure
  (e.g. `fig19`) and `<series>` names the dataset. Examples:
  - `fig03_dsmc.csv` — DSMC centerline density (S0 = 2), Fig. 3
  - `fig19_dsmc.csv` — DSMC centerline density, Kn = 100, Fig. 19
  - `fig25_dsmc_kn100.csv`, `fig25_dsmc_kn0p1.csv`,
    `fig25_dsmc_kn0p01.csv` — the three DSMC mass-flux curves, Fig. 25
  - contour figures: one CSV per digitized contour polyline, e.g.
    `fig06_dsmc_0p001.csv` (the DSMC n/n0 = 0.001 line of Fig. 6)
- Content: two comma-separated columns `x,y` with one header row.
  x/y are in the figure's plotted units (X/D, Z/D, theta in degrees,
  normalized quantity values).
- Series suffixes are prettified for legends: `dsmc` -> `DSMC`,
  `kn0p1` -> `Kn=0.1`.
- Lower-half-plane series in the split contour figures (Figs. 6-8,
  11-18) should be digitized with positive Z/D values taken from the
  paper's lower half; enter them as printed (negative y) — overlays are
  drawn exactly as given.

Each figure script's docstring lists the exact filenames it looks for.
Files that are absent are silently skipped.
