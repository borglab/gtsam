# Opt-in Optimizer Timing Implementation Plan

**Goal:** Merge `cuda-gnc` into the direct sparse solver branch and make GNC
and direct sparse detailed timing default-off without removing correctness
synchronization.

**Design reference:**
`docs/superpowers/specs/2026-07-20-opt-in-optimizer-timing-design.md`

## Task 1: Establish tests

- Change the direct sparse parameter-default test to require
  `collectTiming == false`.
- Add GNC tests proving the default path returns zero/empty timing and the
  explicit opt-in path records timing.
- Run the focused tests and confirm the new expectations fail before changing
  production defaults.

## Task 2: Merge reviewed CUDA GNC fixes

- Merge `cuda-gnc` without rewriting history.
- Resolve `timeCudaSFMBAL.cpp` by retaining all direct sparse benchmark paths
  and applying CUDA LM detailed profiling only for `--profile`.
- Inspect the merge result before adding new timing behavior.

## Task 3: Implement opt-in defaults

- Add `GncParams::enableTiming = false` and gate all GNC `steady_clock` reads
  and timing-vector writes.
- Change direct sparse `collectTiming` to default `false`; retain its existing
  internal event and synchronization gates.
- Explicitly enable GNC and sparse timing in the benchmark paths that consume
  detailed timing.

## Task 4: Verify

- Build affected targets.
- Run focused timing tests, then the merged CUDA/GNC/outlier/sparse regression
  set.
- Review the final diff for accidental synchronization or benchmark-mode loss.
