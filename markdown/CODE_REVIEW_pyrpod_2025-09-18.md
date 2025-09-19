# PyRPOD `pyrpod/` Code Review — DRY and Maintainability

Date: 2025-09-18
Scope: All Python under `pyrpod/` (mission, rpod, vehicle, mdao, orbital, plume, util)

## What I looked at
- Packages: mission, rpod, vehicle, plume, mdao, orbital, util
- Representative files:
  - `mission/`: `MissionPlanner.py`, `six_dof_dynamics.py`, `flight_eval.py`, `MissionEnvironment.py`
  - `rpod/`: `RPOD.py`, `JetFiringHistory.py`
  - `vehicle/`: `Vehicle.py`, `VisitingVehicle.py`, `LogisticsModule.py`
  - `mdao/`: `SweepConfig.py`
  - `plume/`: `RarefiedPlumeGasKinetics.py`, `IsentropicExpansion.py`
  - `util/`: `io/file_print.py`, `stl/transform_stl.py`

## Highlights (what’s working well)
- Clear domain structure: mission, rpod, vehicle, plume, mdao are separated logically.
- Good docstrings throughout, with parameter/return descriptions that help future readers.
- Config-driven design via `configparser` keeps cases flexible without code changes.
- Useful utilities:
  - JFH I/O helpers in `util/io/file_print.py` with tests referring to them.
  - `Vehicle.convert_stl_to_vtk*` centralizes mesh-to-VTK conversion.
- Physics/maths organization:
  - Plume models in `plume/` are self-contained and reasonably documented.
  - Orbital calculations are isolated in `orbital/HohmannTransfer.py`.
- Encapsulation:
  - `MissionEnvironment` wraps case setup (vehicles, JFH) and offers logging slots for results.
  - `MissionPlanner` composes submodules and suggests a clean orchestration point.

## DRY and maintainability concerns

1) Duplicated helpers and logic
- rotation_matrix_from_vectors duplicated
  - Defined in `rpod/RPOD.py` and `rpod/JetFiringHistory.py`, and used in many places. This belongs in a single utility (e.g., `pyrpod/util/math/transform.py`).
- Near-identical STL/mesh routines
  - Similar logic in `RPOD.graph_jfh` and `RPOD.visualize_sweep` (build VV mesh, rotate/translate, build plume meshes, optional clusters, save STL). This can be a single function with options.
  - Multiple places create results directories and save assets using repeated os.path checks; could be factored into a small FS helper.
- Repeated “plume transformation pipeline”
  - Sequence: DCM for thruster, rotate plume, rotate with VV orientation, translate by VV position, then cluster offset, then thruster exit. This appears in multiple methods; factor into a utility.
- Trans/rot performance calculations repeated
  - `mission/six_dof_dynamics.py` and `mission/flight_eval.py` both have `calc_trans_performance` with identical logic. Consolidate into dynamics or a helper.

2) Mixed responsibilities and large classes
- `rpod/RPOD.py` is very large and mixes:
  - Visualization (matplotlib creation, saving images)
  - STL composition and file I/O
  - Physics (1D approach, delta-v computation, time multiplier)
  - Plume strikes computation and export
  - This should be split into smaller services/modules. As-is, it’s hard to test and evolve.

3) Inconsistent data types and old NumPy patterns
- Use of `np.matrix` (discouraged) vs `np.array`. Mixing `np.matrix` vs `np.array` with `.A` indexing in `file_print.print_JFH`. Standardize to `np.array` everywhere, update consumers accordingly.
- Rotations and DCMs passed as nested Python lists intermixed with arrays; should be `np.ndarray` consistently.

4) Error handling/logging gaps
- Empty try/except that silently sets None or returns (e.g., many config loads) with no logs. This makes debugging difficult in production runs.
- Many commented-out print statements, but no structured logging. Prefer logging library.
- File and directory writes don’t catch filesystem errors; return values/status are unused.

5) Hard-coded paths and relative resources
- Some methods refer to `../stl/...` instead of case-dir bound paths (e.g., `VisitingVehicle.initiate_plume_mesh` uses `../data/stl/mold_funnel.stl`), while others correctly use `self.case_dir`. This inconsistency will break when run from different cwd.

6) Configuration and “stateful object” coupling
- Many classes re-read `config.ini` individually; `MissionEnvironment` could be the single source.
- `LogisticsModule` inherits from `VisitingVehicle` but duplicates some constructor/config reading; `Vehicle` also reads config independently. Consolidate source of truth for config and case_dir in `MissionEnvironment` and pass references.

7) API clarity
- Some methods return different shapes conditionally (e.g., `set_strike_fields` returns different tuples based on kinetics mode), which complicates usage.
- `RPOD.jfh_plume_strikes` returns `firing_data` but also writes VTK files and updates internal state — consider a clearer separation of compute vs side-effects.

8) Testability and separation of concerns
- Many methods do heavy I/O (read/write files, create directories) in the same code paths as computations, making unit testing hard.
- Compute functions should accept inputs and return data; persistence and visualization should be separate adapters.

9) Style/naming consistency
- CamelCase and snake_case mixed for file/class/method names in places.
- Some typos: “Caculated”, “inlcude”, “instatiated”, etc. Minor but present.

## Concrete examples of duplication and opportunities

- rotation_matrix_from_vectors
  - Found in both `RPOD.py` and `JetFiringHistory.py`. Centralize in utils; update imports.
- Nearly identical loops in `graph_jfh` and `visualize_sweep`
  - Construct VV mesh, thrusters, clusters, rotate/translate, aggregate, save.
- `calc_trans_performance` in two places
  - Same variables and math in `six_dof_dynamics.py` and `flight_eval.py`.

## Phased refactoring plan

### Phase 0: Safety net and decisions (short)
- Add a lightweight logging setup (logging.getLogger("pyrpod")) and start replacing print with logger.debug/info/warning.
- Decide on numpy array standard for rotations (drop np.matrix); encode in a short style guide doc in `pyrpod/README_new.md`.
- Add a tiny runtime guard for case_dir existence and helpful error if missing.

### Phase 1: DRY quick wins (low risk, local changes)
- Extract `rotation_matrix_from_vectors` to `pyrpod/util/math/transform.py` and update imports in `rpod/RPOD.py` and `rpod/JetFiringHistory.py`.
- Factor out a small filesystem helper: `pyrpod/util/fs.py` with `ensure_dir(path)`, `ensure_parent_dir(file_path)`.
- Deduplicate `calc_trans_performance`: keep once in `SixDOFDynamics` and have `FlightEvaluator` call into it (or import a mission.util).
- Standardize to `np.array`:
  - Replace uses of `np.matrix` in `VisitingVehicle.transform_plume_mesh`, `JetFiringHistory`, and RPOD rotation construction; adjust downstream code that depends on `.A`.
  - Update `util/io/file_print.py` to handle arrays using `rot[i][j][k]` safely.
- Fix hard-coded relative paths:
  - Replace `../data/stl/...` and `../stl/...` with `self.case_dir + 'stl/...'` consistently.

Acceptance for Phase 1:
- Unit tests still pass (including `tests/rpod/rpod_unit_test_03.py` which uses print_JFH).
- No functional change in outputs except path normalization and stable logs.
- No use of np.matrix remains.

### Phase 2: Separate compute from I/O/visualization (medium)
- Split `rpod/RPOD.py` into:
  - `rpod/geometry.py` (mesh building: VV/cluster/thruster plume composition, transform pipelines)
  - `rpod/impingement.py` (plume strikes computation, returns arrays without writing)
  - `rpod/io.py` (VTK/STL writers and result directory setup)
  - `rpod/one_d_approach.py` (1D kinematics and mass calc, JFH generation utilities)
  - Keep a thin `RPOD` orchestrator that wires these pieces.
- In `MissionPlanner` and `MissionEnvironment`, centralize config and case_dir; stop re-reading configs in Vehicle classes. Pass environment to components that need it.
- Make `MissionEnvironment` the canonical provider of `vv` and `tv`; remove duplicated config readers in `Vehicle` so they accept an environment or config object in constructor.

Acceptance for Phase 2:
- RPOD flows still run for current cases (existing case dirs).
- Graph generation and STL/VTK export still produced with same filenames.
- Compute functions can be called without touching filesystem (add 1-2 new unit tests around pure functions).

### Phase 3: API cleanups and stronger typing (medium)
- Ensure consistent return types/signatures:
  - `set_strike_fields` and `set_plume_strike_fields` return consistent namedtuple/dataclass (e.g., StrikeFields with optional physics arrays set to None if not used).
- Introduce dataclasses for common structures:
  - Thruster, Cluster, FiringEvent (replace raw dicts with typed fields).
  - This improves readability and reduces key-typo bugs.
- Simplify thruster grouping config parsing: return validated structures, add small schema validation/errors with helpful messages.

Acceptance for Phase 3:
- No breaking changes to external behavior; internal code compiles and runs with new types.
- Minimal adapters added where dicts were expected.

### Phase 4: Testing and docs (ongoing)
- Add unit tests for:
  - `rotation_matrix_from_vectors` (edge case: identical vectors)
  - Plume transformation pipeline ordering
  - `calc_trans_performance` with simple inputs
- Add docstrings and a CONTRIBUTING.md style snippet on arrays vs matrices, path handling, and config expectations.

### Phase 5: Optional larger improvements
- Consider a “ResultsWriter” abstraction to capture VTK, CSV, images without littering code paths.
- Consider serialization of intermediate computed results to enable post-processing without re-running simulation.
- Explore using a geometry/transform helper library or unify with a small internal matrix utility wrapping numpy.

## Specific, actionable items list (mapping to code)
- Create `pyrpod/util/math/transform.py` and move `rotation_matrix_from_vectors`.
- Create `pyrpod/util/fs.py` with `ensure_dir(path)` and `ensure_parent_dir(file_path)`.
- Replace:
  - All os.path.isdir/os.mkdir blocks with ensure_dir.
  - All `np.matrix` usage with `np.array`.
- Merge `calc_trans_performance` into one location; import and reuse in `FlightEvaluator`.
- Normalize all STL path references to `self.environment.case_dir`-anchored paths.
- In `file_print.py`, remove reliance on `.A` and assume `rot` is ndarray.
- In `MissionEnvironment`, ensure single config read and pass environment into `VisitingVehicle` and `TargetVehicle` so they don’t re-read `config.ini`.

## Positive impact
- Less duplication, easier to update math/transform once.
- Clearer module responsibilities, smaller files, easier testing.
- Consistent path handling and logging make runs more robust in different environments.
- Moving to np.array reduces surprises and aligns with modern NumPy usage.

## Risks and mitigations
- Changing from np.matrix to np.array can break code expecting `.A` or 2D coercion. Mitigation: change all sites in one sweep and add small tests.
- Centralizing config in `MissionEnvironment` requires touching constructors; do it compatibly (allow passing environment or case_dir, default to current behavior initially).
- Splitting RPOD will require updating imports. Do it incrementally with shims (keep old class but delegate to new modules internally), then clean up.

## Closing summary
- The codebase is neatly organized by domain and already has decent docstrings.
- The main opportunities are consolidating repeated math/IO snippets, modernizing arrays, and separating compute from IO-heavy orchestration.
- The phased plan starts with low-risk DRY fixes and logging, then moves toward clearer APIs and structure without changing external behavior.
