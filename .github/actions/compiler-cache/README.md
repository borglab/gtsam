# Compiler cache policy

Native Linux (including ARM64 and special configurations) and macOS use GTSAM's
existing automatic ccache integration. Windows Debug/Release and the two nightly
Python extras use explicit sccache launchers. Python's `--sccache` option disables
GTSAM's automatic ccache hook so the launchers cannot stack. Windows Debug disables
PCH and selects embedded debug information (`/Z7`) with CMP0141 NEW; it still builds
and runs the same tests with debug information and consistency checks.

The composite action has a `setup` operation before configuration and a `finish`
operation after build/tests. Its only cached path is under `RUNNER_TEMP`, outside
`build/`, which `unix.sh` removes. It never restores generated files or test results.
The finish step reports hits, misses, errors/uncacheable requests, and actual disk
size in the job log and summary. `timed.sh` reports wall time and the original exit
status for build and test commands, including failures.

Namespaces include runner OS/architecture, actual C/C++ compiler binary hashes,
compiler IDs/versions/targets, system architecture, OS release (Linux), selected
SDK/Xcode or MSVC toolset, CMake/cache-tool version, lane, build type, environment
compiler flags, and CMake/helper configuration hashes. There is no cross-toolchain,
cross-configuration, or cross-architecture fallback. The compiler cache itself
still checks source and included-header content.

Archives have a 256 MiB local cache limit and immutable weekly keys. The first
successful trusted build in a week publishes a snapshot; later builds restore it
but do not produce an archive per commit. New entries from those later builds
become persistent at the next weekly rotation. Old snapshots remain subject to
GitHub's cache retention and repository-wide eviction policy; the bound is per
lane/snapshot, not a guarantee about the repository's total quota. Debug objects
may exceed a lane's capacity, so cache coverage and performance need measurement.

Only successful `push`, `schedule`, or `workflow_dispatch` builds of
`borglab/gtsam` at `refs/heads/develop` publish archives. PRs can restore trusted
base-branch caches through GitHub's cache scope rules, but never save archives.
The sccache lanes use the local disk backend; `actions/cache/save` is their only
persistent writer. Neither `pull_request_target` nor a same-repository feature
branch is a trusted write context.

The `Compiler cache validation` workflow is manual-only and runs all native lanes
and the two Python extras when dispatched. Its small C/C++ fixture checks both
cache tools with a cold build, an archive-restored warm build after deleting
`build/`, and a changed-source build. CTest runs on every pass. It also checks
namespace partitioning. The normal PR CI remains in place, and the working vcpkg
integration is unchanged.

A draft PR cannot prove publication or warm restoration from the new trusted
`develop` namespace. After merge, compare the first trusted population run and a
subsequent run for archive restoration, hits/misses/errors, size, build time and
test time. Do not infer a speedup from a cache hit percentage alone. Validate the
next weekly rollover as well. These checks require trusted branch execution; do
not widen cache-write permissions to make a PR appear warm.
