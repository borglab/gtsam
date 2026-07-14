# Direct Sparse Jacobian GPU Prototype Results

## Outcome

The prototype validates the main architectural hypothesis. An ordinary
`NonlinearFactorGraph` can retain GTSAM's existing CPU/TBB factor
linearization semantics, stream temporary per-factor `JacobianFactor` results
directly into a cached scalar CSR layout, and use a persistent GPU sparse
normal-equation solver without constructing a complete
`GaussianFactorGraph`.

On the two measured BAL problems, the generic sparse-Jacobian CUDA optimizer
was 3.37x and 2.02x faster than ordinary GTSAM LM. It remained 10.35x and
12.01x slower than the domain-specific CUDA SFM optimizer. The isolated
streaming path was 3.16x and 2.64x faster than constructing a complete
`GaussianFactorGraph` and then packing the identical CSR layout.

The dominant remaining cost was not steady-state H2D or D2H transfer. One-time
cuDSS analysis was the largest measured stage, followed by persistent sparse
setup and symbolic-plan construction. Direct factor emission is therefore not
the best immediate next milestone unless analysis and structure are first
reused across enough solves to amortize those costs.

## Benchmark environment

| Item | Configuration |
| --- | --- |
| GPU | NVIDIA A100 80GB PCIe |
| NVIDIA driver | 580.159.03 |
| CUDA toolkit | 13.0.88 |
| cuDSS | 0.8.0.10 |
| CPU parallelism | TBB enabled (`GTSAM_USE_TBB`) |
| Branch | `codex/direct-sparse-jacobian` |
| Measured revision | `dee67144b` and the preceding prototype commits |
| Date | 2026-07-14 |

Each optimizer path received one untimed warm-up and was then reported as an
individual measured run. Each packing path received one untimed warm-up and
ten measured repetitions. The CPU, specialized CUDA, and generic CUDA
optimizers used the same input graph, initial values, and LM settings.

The benchmark enforced the shared final-objective tolerance

```text
1e-8 * max(1, abs(CPU objective),
              abs(specialized objective),
              abs(generic objective))
```

All reported optimizer comparisons passed this check.

## Problem sizes

The prototype forms a full scalar CSR normal matrix in this milestone.

| Dataset | Factors | Residual rows | Scalar columns | `J.nnz` | `H.nnz` |
| --- | ---: | ---: | ---: | ---: | ---: |
| `dubrovnik-16-22106-pre.txt` | 83,718 | 167,436 | 66,462 | 2,009,232 | 4,721,022 |
| `dubrovnik-135-90642-pre.txt` | 553,336 | 1,106,672 | 273,141 | 13,280,064 | 30,706,857 |

## Optimizer correctness and end-to-end time

The objective values below are printed at the benchmark's displayed
precision. CPU, specialized CUDA, and generic CUDA results agreed within the
relative tolerance above.

| Dataset | Final objective, all paths | CPU LM | Specialized CUDA SFM LM | Generic sparse CUDA LM | Generic internal wall | CPU / generic | Generic / specialized |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| BAL-16 | 18,034.1031 | 2.74838974 s | 0.07891507 s | 0.816404882 s | 0.804601432 s | 3.37x | 10.35x |
| BAL-135 | 378,041.33 | 6.99151703 s | 0.287682287 s | 3.45414201 s | 3.40234702 s | 2.02x | 12.01x |

The generic BAL-16 run accepted five steps in five lambda attempts and
performed five outer linearizations. The BAL-135 run accepted two steps in
three attempts and performed three outer linearizations. Both runs performed
cuDSS analysis once.

The external generic time includes optimizer construction and `optimize()`.
The internal wall time is the optimizer's profiled region, so the small
difference is expected and should not be interpreted as an unmeasured GPU
stage.

## Generic optimizer stage profile

These timings are cumulative over one complete optimizer run. The streaming
row includes all outer linearizations; factor-and-solve includes all lambda
attempts. Profile fields can overlap: in particular, detailed transfer and
structure events occur inside persistent setup, and the mandatory cuDSS
`DATA_INFO` host boundary overlaps the factor-and-solve interval. The rows must
not be summed to reconstruct total wall time.

| Stage | BAL-16 | BAL-135 |
| --- | ---: | ---: |
| Generic optimizer internal wall | 0.804601432 s | 3.40234702 s |
| Symbolic CSR plan | 0.060030986 s | 0.304803649 s |
| Persistent sparse setup wall | 0.108047978 s | 0.53820707 s |
| Factor linearization + CSR pack wall | 0.072884349 s over 5 outers | 0.246935007 s over 3 outers |
| First cuDSS analysis | 0.288626068 s | 1.73558826 s |
| cuDSS factorization + solve | 0.0105512961 s | 0.0961607704 s |
| CPU `Values::retract` | 0.066766622 s | 0.106727262 s |

cuDSS analysis alone consumed about 36% of the BAL-16 internal wall time and
51% of the BAL-135 internal wall time. It was far larger than repeated numeric
factorization and solve on both problems.

## Transfer accounting

Pattern H2D and setup D2H are one-time initialization traffic. Numeric H2D is
cumulative over all outer linearizations, and attempt D2H is cumulative over
all lambda attempts.

| Transfer stage | BAL-16 | BAL-135 |
| --- | ---: | ---: |
| Pattern H2D | 28,122,464 B in 0.0144773116 s | 182,559,508 B in 0.073300738 s |
| Numeric `J.values` + `b` H2D | 87,066,720 B in 0.00628438401 s | 345,281,664 B in 0.0224511037 s |
| Setup pattern D2H | 38,299,880 B in 0.00665641618 s | 247,839,992 B in 0.0323957443 s |
| Attempt delta + model scalars D2H | 2,658,560 B in 0.000384704001 s | 6,555,432 B in 0.00050192 s |
| Total H2D | 115,189,184 B | 527,841,172 B |
| Total D2H | 40,958,440 B | 254,395,424 B |

Steady numeric H2D took 6.28 ms on BAL-16 and 22.45 ms on BAL-135 across the
complete optimizer runs. Attempt D2H remained below 0.51 ms. These
measurements support the original observation that layout conversion and
sparse-solver lifecycle costs, rather than raw PCIe copies, are the more useful
optimization targets.

## Streaming versus complete Gaussian graph packing

Host clearing was timed independently and is excluded from the combined path
comparison.

| Dataset | Direct streaming mean | Streaming host clear | `graph.linearize()` mean | Identical CSR pack mean | Gaussian graph + pack | Combined / streaming |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| BAL-16 | 0.0057263044 s | 0.0016475184 s | 0.0062824775 s | 0.0118304851 s | 0.0181130448 s | 3.16x |
| BAL-135 | 0.0562963737 s | 0.0123786008 s | 0.0690524287 s | 0.079826092 s | 0.148878694 s | 2.64x |

The direct path still invokes each factor's existing virtual `linearize()` and
therefore still creates a temporary Gaussian factor. Its gain comes from
consuming that object immediately into exclusive preplanned CSR rows, allowing
linearization and scatter to run through the same TBB scheduling pass, and
never retaining the complete `GaussianFactorGraph`. The diagnostic path pays
separately for complete graph construction and a later traversal of the same
CSR scatter logic.

This is strong evidence that cached layout and streaming remove a substantial
part of the conversion/object overhead. It is not evidence that all temporary
factor creation has been eliminated. Streaming remains a material repeated
stage, but it is smaller than the one-time analysis and setup costs in these
complete runs.

## Bottleneck interpretation

The measurements separate three different performance questions:

1. **Does direct streaming address the reported layout bottleneck?** Yes. It
   reduced the isolated Gaussian-graph-plus-pack path by 3.16x on BAL-16 and
   2.64x on BAL-135.
2. **Does a general sparse CUDA optimizer beat ordinary CPU LM here?** Yes. It
   was 3.37x and 2.02x faster on the two measured problems while retaining the
   ordinary factor API and objective.
3. **Does this general path approach the specialized CUDA SFM path?** No. The
   specialized path remained 10.35x and 12.01x faster than the generic path.

For the generic path, one-time symbolic work dominates. cuDSS analysis is the
largest single measured stage, while sparse setup and CPU plan construction
are also substantial. By contrast, the per-outer numeric upload and the
per-attempt result download are small. The full `H = J^T J` representation and
its discovery/validation lifecycle also make setup and memory traffic larger
than an upper-triangle-only design would require.

## Recommendation

Do not make direct `NoiseModelFactor` emission the immediate next milestone.
It can remove the remaining temporary `JacobianFactor` construction and
scatter overhead, but that work targets a smaller stage than cuDSS analysis on
these runs. It also creates a new factor-extension interface that must preserve
whitening, robust noise, constrained-factor rejection, and custom-factor
semantics.

The next work should proceed in this order:

1. Cache and reuse the symbolic plan, GPU structure, and cuDSS analysis across
   repeated solves with unchanged topology and dimensions. This also provides
   the most useful prerequisite for later incremental updates.
2. Reduce sparse setup and storage, especially by evaluating upper-triangle-only
   normal-equation generation instead of constructing the full CSR Hessian.
3. Reduce state-management cost with persistent device-side values and a less
   expensive retract/update path.
4. After those costs are amortized, add direct writers for the hottest built-in
   factors and measure them against the existing compatibility path. Keep the
   virtual `linearize()` route for custom and unsupported factors.

Direct emission becomes a stronger candidate if repeated or incremental solves
successfully amortize analysis and setup until factor evaluation and packing
become dominant.

## Scope retained for follow-up

The measured milestone does not implement:

- direct nonlinear-factor writers;
- compact batch-Jacobian support;
- upper-triangle-only Hessian generation;
- custom GPU factor emitters;
- persistent device `Values`;
- iSAM2 or incremental Bayes-tree integration.

The performance results are from two SFM datasets on one GPU. They validate
the general mechanism and expose its BAL cost structure, but they are not a
claim about every GTSAM graph family or GPU architecture.

## Final verification

The benchmark executable completed the CPU, specialized CUDA, generic CUDA,
direct-streaming, and Gaussian-graph diagnostic paths on both datasets, and
its explicit objective-tolerance checks passed. The final formatted source
also passed:

- cuDSS-enabled `testCudaSparseJacobian`,
  `testCudaSparseLevenbergMarquardt`, `testCudaDeviceValues`, and
  `testCudaSfm`;
- no-cuDSS `testCudaSparseLevenbergMarquardt` and `testCudaDeviceValues`;
- CPU-only `testNonlinearFactorGraph`;
- a fresh TBB-disabled sparse-Jacobian test earlier in the implementation
  matrix; and
- Compute Sanitizer memcheck on `testCudaSparseJacobian`, with an error
  summary of zero.

All focused test executables reported no failures, and `git diff --check`
reported no whitespace errors. The touched C++ and CUDA files were formatted
with clang-format 18.1.8 using the repository's Google style.
