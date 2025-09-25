#!/usr/bin/env bash
# Run a sequence of formatting, linting, and type checks for the PyRPOD project.
# Usage:
#   ./scripts/run_linters.sh            # run checks (won't modify files)
#   ./scripts/run_linters.sh --install  # install tools into the active venv (or system python)
#   ./scripts/run_linters.sh --tests    # run pytest after linters
# Notes:
# - This script is POSIX-compatible bash and can be run from WSL/Git-Bash/macOS/Linux.
# - On Windows PowerShell, run via WSL or Git-Bash. Alternatively, use the PowerShell commands shown in the README.

set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR" || exit 1

INSTALL=false
RUN_TESTS=false
for arg in "$@"; do
  case "$arg" in
    --install) INSTALL=true ;;
    --tests) RUN_TESTS=true ;;
    -h|--help)
      sed -n '1,120p' "$0" ; exit 0 ;;
  esac
done

# Try to activate known venv layout if present (venv-pyrpod)
if [ -f "$ROOT_DIR/venv-pyrpod/bin/activate" ]; then
  # Unix-style venv
  # shellcheck disable=SC1091
  source "$ROOT_DIR/venv-pyrpod/bin/activate"
  echo "Activated virtualenv: venv-pyrpod/bin/activate"
elif [ -f "$ROOT_DIR/venv-pyrpod/Scripts/activate" ]; then
  # Windows-style venv (Git Bash/WSL may still source this)
  # shellcheck disable=SC1091
  source "$ROOT_DIR/venv-pyrpod/Scripts/activate"
  echo "Activated virtualenv: venv-pyrpod/Scripts/activate"
else
  echo "No venv-pyrpod activation script found. Running with current Python environment."
fi

if $INSTALL; then
  echo "Installing/Updating dev tools into current environment (this may take a minute)..."
  python -m pip install --upgrade pip || true
  python -m pip install --upgrade black isort ruff mypy flake8 pytest || true
fi

# Helper to run a command and record its exit status without exiting the script
declare -A results
run_cmd() {
  label="$1"; shift
  echo
  echo "================================================================"
  echo "Running: $label"
  echo "Command: $*"
  echo "----------------------------------------------------------------"
  # Ensure logs directory exists
  LOG_DIR="$ROOT_DIR/logs/linters"
  mkdir -p "$LOG_DIR"
  logfile="$LOG_DIR/${label}.log"

  # Run the command and capture stdout/stderr to logfile
  # Use bash -c to preserve arguments and exit code properly
  ("$@") >"$logfile" 2>&1
  rc=$?
  echo "Full output saved to: $logfile"
  results["$label"]=$rc
  if [ $rc -eq 0 ]; then
    echo "-> $label: PASS"
  else
    echo "-> $label: FAIL (exit code: $rc). See $logfile for details."
  fi
}

# Run checks (check-only so no files are modified)
run_cmd "black-check" python -m black --check .
run_cmd "isort-check" python -m isort --check-only --profile black .
run_cmd "ruff-check" python -m ruff check .
run_cmd "flake8" python -m flake8 .
run_cmd "mypy" python -m mypy pyrpod

if $RUN_TESTS; then
  run_cmd "pytest" python -m pytest -q
fi

# Print summary
echo
echo "================================================================"
echo "Summary"
fail=0
for key in "${!results[@]}"; do
  status=${results[$key]}
  if [ "$status" -eq 0 ]; then
    printf "PASS: %s\n" "$key"
  else
    printf "FAIL: %s (exit code %d)\n" "$key" "$status"
    fail=1
  fi
done

if [ $fail -eq 1 ]; then
  echo
  echo "One or more checks failed. See the output above for details."
else
  echo
  echo "All checks passed. ✅"
fi

exit $fail
