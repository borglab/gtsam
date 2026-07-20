# Opt-in Optimizer Timing Design

## Goal

Merge the reviewed `cuda-gnc` fixes into `codex/direct-sparse-jacobian` and
make detailed timing collection opt-in across the GNC and direct sparse CUDA
optimizers. Normal optimization must avoid measurement-only clock reads, CUDA
events, and synchronization while preserving numerical behavior and required
host/device boundaries.

## Public behavior

- `GncParams::enableTiming` defaults to `false`.
- `CudaSparseLevenbergMarquardtParams::collectTiming` defaults to `false`.
- When disabled, timing result fields remain zero and GNC stores no per-outer
  iteration timing records.
- Benchmark/profile entry points explicitly enable timing before reading or
  printing detailed timing results.
- Existing callers can restore the old instrumentation behavior by setting the
  corresponding flag to `true`.

## Synchronization boundary

The direct sparse device layer already receives `collectTiming` and gates CUDA
event creation, event recording, event harvesting, and the persistent-setup
profiling synchronization. Those gates remain intact and default off.

Synchronizations required to consume downloaded attempt results, inspect a
discovered sparse pattern on the host, report cuDSS status at a host boundary,
or destroy resources safely remain unconditional. They are part of correctness
or lifetime management, not optional profiling.

## Integration

Merge `cuda-gnc` into `codex/direct-sparse-jacobian` with a normal merge commit.
Resolve the expected conflict in `timing/sfm_ba/timeCudaSFMBAL.cpp` by retaining
the direct sparse benchmark modes while adopting the reviewed CUDA LM
`--profile` opt-in behavior. Do not rewrite either branch's history.

The existing stash is not applied because it contains only whitespace changes
and is unrelated to this implementation.

## Tests

- Assert both public timing flags default to `false`.
- Assert disabled direct sparse timing remains entirely zero.
- Assert explicitly enabled direct sparse timing remains finite and populated.
- Assert disabled GNC timing remains zero/empty.
- Assert explicitly enabled GNC timing remains populated.
- Run the affected GNC, CUDA SFM, outlier sampling, sparse Jacobian, and sparse
  CUDA LM tests after the merge.
