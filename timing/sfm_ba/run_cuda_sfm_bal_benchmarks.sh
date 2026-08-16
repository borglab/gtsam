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

for configuration in "${configurations[@]}"; do
  record=$(
    "$benchmark_binary" --cuda-lm --configuration "$configuration" \
      --output-format json --cuda-warmup-file "$dataset" "$dataset"
  )
  if ! grep -Eq '"final_objective":[0-9.eE+-]+' <<<"$record"; then
    echo "Missing finite final objective for $configuration" >&2
    exit 1
  fi
  if ! grep -Fq "\"configuration\":\"$configuration\"" <<<"$record"; then
    echo "Configuration mismatch for $configuration" >&2
    exit 1
  fi
  echo "$record" | tee -a "$results_file"
done

echo "Wrote $results_file" >&2
