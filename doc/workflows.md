# GTSAM GitHub Configuration

This directory contains GitHub-specific configuration, including issue templates and CI workflows.

## Workflows Overview

The `.github/workflows` directory contains definitions for the various CI/CD processes:

-   **`build-linux.yml`**: Main Linux CI. Runs inside the pre-built Docker containers mentioned above to compile and test GTSAM with various compilers (GCC, Clang) and configurations.
-   **`build-macos.yml`**: Compiles and tests GTSAM on macOS runners.
-   **`build-windows.yml`**: Compiles and tests GTSAM on Windows runners using MSVC.
-   **`build-python.yml`**: Verifies the Python wrapper compilation across Linux, macOS, and Windows.
-   **`build-cibw.yml`**: Builds distributable Python wheels using `cibuildwheel`. It builds dependencies (like Boost) from source to ensure ABI compatibility.
-   **`prod-cibw.yml`**: Used for production wheel builds (often triggered on release).
-   **`vcpkg.yml`**: Compiles and tests GTSAM on Linux, macOS, and Windows using `vcpkg` to install all dependencies.
-   **`deploy.yml`**: Handles deployment tasks (e.g., docs, release artifacts).

## Updating vcpkg dependencies

`vcpkg` supports a manifest in the form of `vcpkg.json`, which specifies all the dependencies to install, and a "baseline", which is just a commit hash from https://github.com/microsoft/vcpkg. Instead of upgrading individual library versions which might be incompatible, you upgrade baselines, which are a set of library versions that have been tested to work together. This also means library versions are pinned to the baseline, so the baseline needs to periodically updated to get the latest libraries. Updating is not *strictly* necessary, as old library versions should work for a while, but it can be good to ensure compatibility with the latest libraies. To update the baseline, simply go to https://github.com/microsoft/vcpkg and copy the full commit hash for the version you want (could be HEAD for the day, or if you prefer tags, you can use the commit hash for a tag instead) and update the `builtin-baseline` field in `vcpkg.json` at the root of this repo.

## Docker CI Images (Linux Only)

The **Linux CI** workflow (`build-linux.yml`) relies on pre-built Docker images to ensure consistency and speed up build times. These images are hosted in the [borglab/docker-images](https://github.com/borglab/docker-images) repository.

*Note: macOS and Windows workflows use standard GitHub Actions runners and install dependencies (like Boost) via package managers (Homebrew, Chocolatey) or from source.*

### Building and Updating Images

If you need to update an existing CI image or add a new one (e.g., for a new Ubuntu version or compiler):

1.  **Navigate to the `docker-images` repository:**
    ```bash
    cd ../docker-images/gtsam-ci
    ```
2.  **Add or modify a Dockerfile:**
    -   Follow the naming convention: `ubuntu-<version>-<compiler>-<version>.Dockerfile`.
    -   Base images are defined in `*-base.Dockerfile`.
3.  **Build and Push:**
    Use the provided script to build and push to Docker Hub (requires `borglab` permissions):
    ```bash
    ./build_and_push.sh <dockerhub-username>
    ```

For more details, see the [README in the docker-images repository](https://github.com/borglab/docker-images/blob/main/gtsam-ci/README.md).

## timeSFMBAL benchmark flow

The `timeSFMBAL` benchmark is split across three workflows that coordinate through `workflow_dispatch` inputs and shared GitHub Actions cache keys:

- **`time-sfmbal-benchmark-trigger.yml`**: Watches PR comments. If a comment is exactly `/bench`, it dispatches `time-sfmbal-benchmark.yml` with the PR number.
- **`time-sfmbal-benchmark.yml`** (orchestrator): Resolves PR context (head/base SHAs), decides which worker runs are needed, waits for worker completion, collects cached results, compares base vs head, and posts/updates a PR comment.
- **`time-sfmbal-benchmark-runner.yml`** (worker): Runs on one runner profile (`linux-x64`, `linux-arm64`, or `macos-arm64`) for one target SHA, benchmarks both `GTSAM_WITH_TBB=ON` and `OFF`, then writes result JSONs into cache.

How coordination works:

1. The orchestrator computes six runner specs: 3 runner profiles x 2 roles (`head` and `base`).
2. For each spec, it checks whether both benchmark cache entries already exist (`tbbON` and `tbbOFF`).
3. Only missing specs are dispatched as worker workflow runs; fully cached specs are skipped.
4. The orchestrator waits for dispatched workers, then the `collect` job restores cached JSON files for every `(os, arch, tbb, sha)` matrix entry.
5. The `summarize` job compares restored `head` vs `base` JSON files via `.github/scripts/compare_time_sfmbal_benchmarks.py` and posts a single benchmark comment (updated in-place on reruns).

Caching details:

- **Dataset cache (worker workflow):**
  - Key: `bal-problem-135-90642-pre-txt-bz2-v1`
  - Payload: `examples/Data/problem-135-90642-pre.txt.bz2`
  - Purpose: avoid re-downloading the BAL dataset archive on every worker run.
- **Benchmark result cache (worker workflow):**
  - Key format: `timeSFMBAL-benchmark-v4-<os>-<arch>-tbb<ON|OFF>-<sha>`
  - Payload: one JSON benchmark result per `(os, arch, tbb, sha)`
  - Purpose: reuse previously computed benchmark results for identical commit SHA and runner profile.
- **Cache-aware dispatch (orchestrator):**
  - Worker dispatch is skipped only when both `tbbON` and `tbbOFF` keys exist for that `(os, arch, sha)`.
  - This prevents partial reuse from hiding missing variants.
- **Cache-aware collection (orchestrator):**
  - The `collect` matrix restores exact `head` and `base` keys and stages either JSON files or explicit miss markers.
  - Missing cache entries are surfaced in the generated benchmark report.
