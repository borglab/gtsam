#!/usr/bin/env bash
# Preserve command failure while reporting wall time, including failed tests.
set -uo pipefail
label=$1
shift
start=$SECONDS
"$@"
status=$?
elapsed=$((SECONDS - start))
echo "$label: ${elapsed}s (exit $status)"
if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  echo "- $label: ${elapsed}s (exit $status)" >> "$GITHUB_STEP_SUMMARY"
fi
exit "$status"
