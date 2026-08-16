#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 BAL_DATASET [OUTPUT_DIRECTORY] [BENCHMARK_BINARY]"
  echo "Runs every supported CUDA SFM formulation/backend/ordering row."
}

if [[ $# -lt 1 || $# -gt 3 ]]; then
  usage >&2
  exit 2
fi

dataset=$1
output_directory=${2:-cuda_sfm_benchmark_results}
benchmark_binary=${3:-${CUDA_SFM_BENCHMARK_BINARY:-build-cuda/timing/sfm_ba/timeCudaSFMBAL}}

if [[ ! -f "$dataset" ]]; then
  echo "BAL dataset does not exist: $dataset" >&2
  exit 2
fi
if [[ ! -x "$benchmark_binary" ]]; then
  echo "Benchmark binary is not executable: $benchmark_binary" >&2
  exit 2
fi

mkdir -p "$output_directory"
results_file="$output_directory/results.jsonl"
: >"$results_file"

configurations=(
  schur-dense
  schur-cudss-auto
  schur-cudss-gtsam
  schur-pcg
  full-normal-cudss-auto
  full-normal-cudss-gtsam
  full-normal-pcg
)

reference_objective=
for configuration in "${configurations[@]}"; do
  record=$(
    "$benchmark_binary" --cuda-lm --configuration "$configuration" \
      --output-format json --cuda-warmup-file "$dataset" "$dataset"
  )
  if ! jq -e --arg configuration "$configuration" \
      '.configuration == $configuration and
       (.final_objective | type == "number")' <<<"$record" >/dev/null; then
    echo "Configuration mismatch for $configuration" >&2
    exit 1
  fi
  objective=$(jq -r '.final_objective' <<<"$record")
  if [[ -z "$reference_objective" ]]; then
    reference_objective=$objective
  fi
  tolerance=1e-8
  if [[ "$configuration" == *pcg* ]]; then
    tolerance=1e-3
    if [[ $(jq -r '.pcg_max_iteration_hits' <<<"$record") != 0 ]]; then
      echo "PCG hit the iteration cap for $configuration" >&2
      exit 1
    fi
    if [[ $(jq -r '.pcg_breakdown_count' <<<"$record") != 0 ]]; then
      echo "PCG broke down for $configuration" >&2
      exit 1
    fi
    if [[ $(jq -r '.pcg_converged' <<<"$record") != true ]]; then
      echo "PCG did not converge for $configuration" >&2
      exit 1
    fi
  fi
  relative_error=$(awk -v actual="$objective" -v reference="$reference_objective" \
    'BEGIN { d = reference < 0 ? -reference : reference; if (d < 1) d = 1;
             e = actual - reference; if (e < 0) e = -e;
             printf "%.17g", e / d }')
  objective_pass=$(awk -v error="$relative_error" -v tolerance="$tolerance" \
    'BEGIN { print error <= tolerance ? "true" : "false" }')
  if [[ "$objective_pass" != true ]]; then
    echo "Objective mismatch for $configuration: relative error $relative_error > $tolerance" >&2
    exit 1
  fi
  expected_matrix_free=false
  if [[ "$configuration" == *pcg* ]]; then
    expected_matrix_free=true
  fi
  if [[ $(jq -r '.matrix_free' <<<"$record") != "$expected_matrix_free" ]]; then
    echo "Representation metadata mismatch for $configuration" >&2
    exit 1
  fi
  if [[ "$configuration" == *gtsam ]] &&
     [[ $(jq -r '.user_ordering_applied' <<<"$record") != true ]]; then
    echo "GTSAM ordering was not applied for $configuration" >&2
    exit 1
  fi
  record=$(jq -c --argjson reference "$reference_objective" \
    --argjson relative_error "$relative_error" --argjson tolerance "$tolerance" \
    '. + {reference_objective: $reference,
          objective_relative_error: $relative_error,
          objective_tolerance: $tolerance,
          correctness_pass: true}' <<<"$record")
  echo "$record" | tee -a "$results_file"
done

echo "Wrote $results_file" >&2
