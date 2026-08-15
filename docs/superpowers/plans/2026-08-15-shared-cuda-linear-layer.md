# Shared CUDA Linear Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build one prepared-system CUDA linear layer for dense Cholesky, cuDSS, and PCG; migrate the general CUDA LM and SFM LM to it; add GTSAM scalar ordering to cuDSS; and support dense, sparse-direct, and implicit-PCG SFM Schur solves.

**Architecture:** `gtsam/linear/cuda` owns system views/storage, ordering compilation, capability checks, solver sessions, and common statistics. The general frontend remains a Jacobian/normal-equation producer, while SFM owns persistent camera/point block linearization, explicit dense/sparse Schur construction, the implicit Schur operator, and point recovery. LM acceptance, nonlinear error, and formulation-specific timing remain outside the shared linear layer.

**Tech Stack:** C++17, CUDA C++, cuSOLVER DN, cuSPARSE, NVIDIA cuDSS, GTSAM `Ordering`, CMake, CppUnitLite, compute-sanitizer.

---

## File map

New shared linear files:

- `gtsam/linear/cuda/CudaLinearSystem.h`: borrowed dense/sparse/operator and preconditioner interfaces.
- `gtsam/linear/cuda/CudaLinearSolver.h/.cpp`: backend enum, common options/stats, capability validation, and tagged session dispatch.
- `gtsam/linear/cuda/CudaBlockOrdering.h/.cpp`: strict GTSAM-key block layout and scalar-permutation compiler.
- `gtsam/linear/cuda/DeviceSparseSpdSystem.h/.cu`: persistent upper-CSR SPD device storage, formerly `DeviceSparseNormalEquations`.
- `gtsam/linear/cuda/CudaDenseCholeskySolver.h/.cu`: persistent cuSOLVER DN Cholesky backend extracted from SFM.
- `gtsam/linear/cuda/CudssSpdSolver.h/.cpp`: shared cuDSS backend with optional user permutation.
- `gtsam/linear/cuda/CudaPcgSolver.h/.cu`: formulation-independent PCG recurrence over operator/preconditioner interfaces.
- `gtsam/linear/tests/testCudaLinearSolver.cpp`: ordering, validation, dense, cuDSS, PCG, and repeated-update tests.

General frontend files:

- `gtsam/nonlinear/cuda/CudaJacobianNormalOperator.h/.cu`: `J^T(Jx)+lambda Dx` and existing variable-block preconditioner adapter.
- `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h/.cu`: retains problem production/model evaluation and loses solver ownership.
- `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h/.cpp`: common session, backend params, user ordering, and common stats integration.
- `gtsam/nonlinear/cuda/{DeviceSparseNormalEquations,CudssLinearSolver,DevicePcgSolver}.h`: compatibility includes/aliases during source migration.
- `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp` and `testCudaSparseLevenbergMarquardt.cpp`: behavior and ordering regression.

SFM frontend files:

- `gtsam/slam/cuda/CudaSfmSchurProblem.h/.cu`: persistent projection linearization, `U/V/W`, gradients, damping, dense assembly, and recovery.
- `gtsam/slam/cuda/CudaSfmReducedCsrPlan.h/.cpp`: symbolic camera co-visibility graph and stable scalar upper-CSR offsets.
- `gtsam/slam/cuda/CudaSfmSparseSchur.h/.cu`: numerical sparse Schur/rhs assembly.
- `gtsam/slam/cuda/CudaSfmSchurOperator.h/.cu`: implicit Schur apply and 9-by-9 camera-block preconditioner.
- `gtsam/slam/cuda/CudaSfmFullNormalProblem.h/.cu`: prepared full-normal producer used through the common session.
- `gtsam/slam/cuda/CudaSfmDenseSchurSolver.h/.cu`: compatibility facade over the new Schur problem plus dense backend.
- `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h/.cu` and `cuda_sfm.i`: independent formulation/backend parameters, LM base semantics, ordering, stats, and dispatch.
- `gtsam/slam/tests/testCudaSfm.cpp`: dense regression, reduced-plan, cross-backend, full-normal, damping, noise, recovery, and boundary tests.

Build, timing, and documentation files:

- `gtsam/linear/CMakeLists.txt` and `gtsam/linear/tests/CMakeLists.txt`: install/gate shared CUDA headers and tests.
- `timing/cuda_sparse/timeCudaSparseLM.cpp`: auto-order, GTSAM-order, and PCG variants plus common stats.
- `timing/sfm_ba/timeCudaSFMBAL.cpp`: formulation/backend/ordering matrix plus common stats.
- `timing/sfm_ba/run_cuda_sfm_bal_benchmarks.sh`: reproducible matrix runner.
- `docs/CUDA_LINEAR_SOLVERS.md`: ownership, capability matrix, ordering semantics, profiles, and benchmark commands.

## Task 1: Establish common system contracts and capability validation

**Files:**
- Create: `gtsam/linear/cuda/CudaLinearSystem.h`
- Create: `gtsam/linear/cuda/CudaLinearSolver.h`
- Create: `gtsam/linear/cuda/CudaLinearSolver.cpp`
- Modify: `gtsam/linear/CMakeLists.txt`
- Test: `gtsam/linear/tests/testCudaLinearSolver.cpp`
- Modify: `gtsam/linear/tests/CMakeLists.txt`

- [ ] **Step 1: Write failing capability tests**

Add tests which assert this exact matrix and ordering rule:

```cpp
TEST(CudaLinearSolver, CapabilityMatrix) {
  EXPECT(CudaLinearSolverSession::Supports(CudaLinearSolverType::DenseCholesky,
      CudaLinearSystemKind::Dense));
  EXPECT(CudaLinearSolverSession::Supports(CudaLinearSolverType::Cudss,
      CudaLinearSystemKind::Sparse));
  EXPECT(CudaLinearSolverSession::Supports(CudaLinearSolverType::Pcg,
      CudaLinearSystemKind::Operator));
  EXPECT(!CudaLinearSolverSession::Supports(CudaLinearSolverType::DenseCholesky,
      CudaLinearSystemKind::Sparse));
  EXPECT_EXCEPTION(CudaLinearSolverSession::Validate(
      {CudaLinearSolverType::Pcg, true}, CudaLinearSystemKind::Operator),
      std::invalid_argument);
}
```

- [ ] **Step 2: Run the new test and verify the missing API failure**

Run: `cmake --build build --target testCudaLinearSolver -j2`

Expected: compilation fails because `CudaLinearSolverSession` and the common enums do not exist.

- [ ] **Step 3: Add the system contracts**

Define borrowed views and host-dispatched interfaces:

```cpp
enum class CudaSparseTriangle { Upper, Lower };
enum class CudaLinearSystemKind { Dense, Sparse, Operator };

struct CudaDenseSpdSystemView {
  int dimension = 0;
  int leadingDimension = 0;
  double* values = nullptr;
  double* rhs = nullptr;
};

struct CudaSparseSpdSystemView {
  int dimension = 0;
  int nonzeros = 0;
  const int* rowPointers = nullptr;
  const int* columnIndices = nullptr;
  double* values = nullptr;
  double* rhs = nullptr;
  CudaSparseTriangle triangle = CudaSparseTriangle::Upper;
};

class CudaLinearOperator {
 public:
  virtual ~CudaLinearOperator() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double*, double*, cudaStream_t) const = 0;
};

class CudaPreconditioner {
 public:
  virtual ~CudaPreconditioner() = default;
  virtual int dimension() const = 0;
  virtual void apply(const double*, double*, cudaStream_t) const = 0;
};
```

Add `CudaLinearSolverType`, options, stats, strict validation, and an initially empty PIMPL session. Make `gtsam/linear/CMakeLists.txt` install `cuda/*.h` when CUDA is enabled; exclude `testCudaLinearSolver.cpp` when it is disabled.

- [ ] **Step 4: Build and run the capability test**

Run: `cmake --build build --target testCudaLinearSolver -j2 && ./build/gtsam/linear/tests/testCudaLinearSolver`

Expected: all capability tests pass.

- [ ] **Step 5: Commit**

```bash
git add gtsam/linear/cuda gtsam/linear/CMakeLists.txt gtsam/linear/tests
git commit -m "feat: add common CUDA linear contracts"
```

## Task 2: Compile GTSAM block ordering into a strict scalar permutation

**Files:**
- Create: `gtsam/linear/cuda/CudaBlockOrdering.h`
- Create: `gtsam/linear/cuda/CudaBlockOrdering.cpp`
- Modify: `gtsam/linear/tests/testCudaLinearSolver.cpp`

- [ ] **Step 1: Write valid and invalid ordering tests**

Use nonuniform dimensions so the test proves block expansion rather than key renaming:

```cpp
TEST(CudaBlockOrdering, ExpandsKeysToScalars) {
  CudaBlockLayout layout{{X(1), 0, 2}, {L(4), 2, 3}, {X(7), 5, 1}};
  EXPECT(assert_equal(std::vector<int>{2, 3, 4, 0, 1, 5},
      CompileCudaScalarPermutation(layout, Ordering{L(4), X(1), X(7)})));
}

TEST(CudaBlockOrdering, RejectsMissingDuplicateUnknownAndGappedBlocks) {
  const CudaBlockLayout layout{{X(1), 0, 2}, {X(2), 2, 2}};
  EXPECT_EXCEPTION(CompileCudaScalarPermutation(layout, Ordering{X(1)}),
                   std::invalid_argument);
  EXPECT_EXCEPTION(CompileCudaScalarPermutation(layout,
                   Ordering{X(1), X(1)}), std::invalid_argument);
  EXPECT_EXCEPTION(CompileCudaScalarPermutation(layout,
                   Ordering{X(1), X(3)}), std::invalid_argument);
  EXPECT_EXCEPTION(CompileCudaScalarPermutation(
                   {{X(1), 0, 2}, {X(2), 3, 1}}, Ordering{X(1), X(2)}),
                   std::invalid_argument);
}
```

- [ ] **Step 2: Run and observe undefined ordering types**

Run: `cmake --build build --target testCudaLinearSolver -j2`

Expected: compilation fails on `CudaBlockLayout`.

- [ ] **Step 3: Implement strict block and permutation validation**

Expose:

```cpp
struct CudaVariableBlock {
  Key key;
  int scalarOffset;
  int dimension;
};
using CudaBlockLayout = std::vector<CudaVariableBlock>;

GTSAM_EXPORT int ValidateCudaBlockLayout(const CudaBlockLayout& blocks);
GTSAM_EXPORT std::vector<int> CompileCudaScalarPermutation(
    const CudaBlockLayout& blocks, const Ordering& ordering);
```

Sort a copy by `scalarOffset`, require a contiguous partition starting at zero, build a key-to-block map with duplicate rejection, consume every ordered key once, append `[offset, offset + dimension)`, and require output length equal to total dimension.

- [ ] **Step 4: Run CPU-safe ordering tests**

Run: `cmake --build build --target testCudaLinearSolver -j2 && ./build/gtsam/linear/tests/testCudaLinearSolver`

Expected: valid expansion equals `{2,3,4,0,1,5}` and all four invalid cases throw.

- [ ] **Step 5: Commit**

```bash
git add gtsam/linear/cuda/CudaBlockOrdering.* gtsam/linear/tests/testCudaLinearSolver.cpp
git commit -m "feat: compile GTSAM ordering for CUDA solvers"
```

## Task 3: Promote sparse SPD device storage into the linear layer

**Files:**
- Create: `gtsam/linear/cuda/DeviceSparseSpdSystem.h`
- Create: `gtsam/linear/cuda/DeviceSparseSpdSystem.cu`
- Modify: `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`
- Modify: `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.cu`
- Modify: callers found by `rg -l 'DeviceSparseNormalEquations' gtsam timing`
- Modify: `gtsam/linear/tests/testCudaLinearSolver.cpp`

- [ ] **Step 1: Add storage validation and damping-reset tests**

Create a 3-by-3 upper CSR system with diagonal entries present, upload it, save the undamped diagonal, apply lambda 2, restore/apply lambda 5, and download values. Assert the second system contains exactly the lambda-5 diagonal, not lambda-7 accumulated damping. Add invalid row-pointer, out-of-range-column, and missing-diagonal cases.

- [ ] **Step 2: Run and observe the missing shared storage type**

Run: `cmake --build build --target testCudaLinearSolver -j2`

Expected: compilation fails on `DeviceSparseSpdSystem`.

- [ ] **Step 3: Move storage without changing memory behavior**

Rename the implementation and add:

```cpp
CudaSparseSpdSystemView view() {
  return {rows_, nonzeros(), rowPointers_.data(), colIndices_.data(),
          values_.data(), rhs_.data(), CudaSparseTriangle::Upper};
}
void captureUndampedDiagonal(cudaStream_t stream);
void restoreAndAddDiagonal(double lambda,
    const CudaDeviceArray<double>& damping, cudaStream_t stream);
```

Keep upload validation, profiled copy behavior, allocations, and existing damping kernels intact. Turn the old header into `using DeviceSparseNormalEquations = DeviceSparseSpdSystem;` and leave its `.cu` empty except for the compatibility include so downstream source compatibility is preserved.

- [ ] **Step 4: Run shared and existing normal-equation tests**

Run: `cmake --build build --target testCudaLinearSolver testCudaSparseJacobian testCudaSfm -j2 && ./build/gtsam/linear/tests/testCudaLinearSolver && ./build/gtsam/nonlinear/tests/testCudaSparseJacobian --run_test=DeviceSparseNormalEquations && ./build/gtsam/slam/tests/testCudaSfm --run_test=DeviceSparseNormalEquations`

Expected: shared storage tests pass and existing matching tests either pass or report no selected test without a crash.

- [ ] **Step 5: Commit**

```bash
git add gtsam/linear/cuda/DeviceSparseSpdSystem.* gtsam/nonlinear/cuda/DeviceSparseNormalEquations.* gtsam
git commit -m "refactor: move sparse SPD storage to linear CUDA"
```

## Task 4: Extract the current dense cuSOLVER algorithm as a shared backend

**Files:**
- Create: `gtsam/linear/cuda/CudaDenseCholeskySolver.h`
- Create: `gtsam/linear/cuda/CudaDenseCholeskySolver.cu`
- Modify: `gtsam/slam/cuda/CudaSfmDenseSchurSolver.cu`
- Modify: `gtsam/linear/tests/testCudaLinearSolver.cpp`
- Modify: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add a dense SPD unit solution and capture the SFM baseline**

Solve column-major `[[4,1],[1,3]] * x = [1,2]` and require `x = [1/11, 7/11]` within `1e-12`. In `testCudaSfm.cpp`, retain the current small BAL dense-Schur delta and objective as a golden pre-refactor comparison with tolerance `1e-9`.

- [ ] **Step 2: Run the unit test before extraction**

Run: `cmake --build build --target testCudaLinearSolver -j2`

Expected: compilation fails because `CudaDenseCholeskySolver` is absent.

- [ ] **Step 3: Move the dense solver implementation byte-for-byte where possible**

Expose a persistent backend:

```cpp
class GTSAM_EXPORT CudaDenseCholeskySolver {
 public:
  void analyze(int maximumDimension, cudaStream_t stream = nullptr);
  void solve(CudaDenseSpdSystemView system,
             CudaDeviceArray<double>* solution,
             cudaStream_t stream = nullptr,
             CudaLinearSolveStats* stats = nullptr);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
```

Move `cusolverDn` handle creation, `potrf_bufferSize`, workspace reuse, `potrf`, `potrs`, and device-info checking from `SolveDenseCameraSystemOnDevice`. Do not transpose, repack, change triangle selection, or add host copies. Replace the SFM-local call with the shared backend.

- [ ] **Step 4: Verify exact backend and dense-SFM regression**

Run: `cmake --build build --target testCudaLinearSolver testCudaSfm -j2 && ./build/gtsam/linear/tests/testCudaLinearSolver && ./build/gtsam/slam/tests/testCudaSfm`

Expected: the 2-by-2 solution passes and all existing dense SFM tests retain their prior objective/delta tolerances.

- [ ] **Step 5: Commit**

```bash
git add gtsam/linear/cuda/CudaDenseCholeskySolver.* gtsam/linear/tests/testCudaLinearSolver.cpp gtsam/slam/cuda/CudaSfmDenseSchurSolver.cu gtsam/slam/tests/testCudaSfm.cpp
git commit -m "refactor: extract shared CUDA dense Cholesky"
```

## Task 5: Move cuDSS and apply the real GTSAM scalar permutation

**Files:**
- Create: `gtsam/linear/cuda/CudssSpdSolver.h`
- Create: `gtsam/linear/cuda/CudssSpdSolver.cpp`
- Modify: `gtsam/nonlinear/cuda/CudssLinearSolver.h`
- Modify: `gtsam/nonlinear/cuda/CudssLinearSolver.cpp`
- Modify: `gtsam/linear/tests/testCudaLinearSolver.cpp`

- [ ] **Step 1: Add repeated-numeric and user-permutation tests**

Use one fixed 4-by-4 upper CSR pattern. Solve two distinct SPD numerical value sets without reanalysis and compare with CPU Eigen. Solve the first set once with automatic ordering and once with scalar permutation `{2,3,0,1}`; assert equal solutions, `analysisCount == 1`, and `userOrderingApplied == true` only for the latter. Add wrong-length, duplicate, and out-of-range permutation rejection.

- [ ] **Step 2: Run and observe missing ordered analysis overload**

Run: `cmake --build build --target testCudaLinearSolver -j2`

Expected: compilation fails because `analyze(..., permutation, ...)` is absent.

- [ ] **Step 3: Move cuDSS ownership and set `CUDSS_DATA_USER_PERM` before analysis**

Expose:

```cpp
void analyze(const DeviceSparseSpdSystem& system,
             CudaDeviceArray<double>* solution,
             const std::optional<std::vector<int>>& scalarPermutation,
             cudaStream_t stream = nullptr);
```

Validate a complete bijection over `[0,n)`. After matrix/data objects exist and before `CUDSS_PHASE_ANALYSIS`, execute:

```cpp
if (scalarPermutation) {
  CUDSS_CHECK(cudssDataSet(handle, data, CUDSS_DATA_USER_PERM,
      scalarPermutation->data(), scalarPermutation->size() * sizeof(int)));
}
```

Keep CSR unpermuted, rely on cuDSS RHS/solution handling, reject incompatible explicit reordering configurations, cache the analyzed pattern/permutation identity, and increment common analysis/factor/solve stats. Make old nonlinear headers forwarding includes and type aliases.

- [ ] **Step 4: Run ordered cuDSS tests under sanitizer**

Run: `cmake --build build --target testCudaLinearSolver -j2 && compute-sanitizer --tool memcheck --error-exitcode=99 ./build/gtsam/linear/tests/testCudaLinearSolver`

Expected: automatic and requested-order solutions match; stats prove the user order was applied; no sanitizer errors occur.

- [ ] **Step 5: Commit**

```bash
git add gtsam/linear/cuda/CudssSpdSolver.* gtsam/nonlinear/cuda/CudssLinearSolver.* gtsam/linear/tests/testCudaLinearSolver.cpp
git commit -m "feat: add GTSAM user ordering to shared cuDSS"
```

## Task 6: Separate PCG recurrence from the general Jacobian operator

**Files:**
- Create: `gtsam/linear/cuda/CudaPcgSolver.h`
- Create: `gtsam/linear/cuda/CudaPcgSolver.cu`
- Create: `gtsam/nonlinear/cuda/CudaJacobianNormalOperator.h`
- Create: `gtsam/nonlinear/cuda/CudaJacobianNormalOperator.cu`
- Modify: `gtsam/nonlinear/cuda/DevicePcgSolver.h`
- Modify: `gtsam/nonlinear/cuda/DevicePcgSolver.cu`
- Modify: `gtsam/linear/tests/testCudaLinearSolver.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`

- [ ] **Step 1: Add a formulation-independent PCG unit test**

Implement a test-only device dense operator/preconditioner for the same 2-by-2 SPD matrix, run PCG from zero with tolerance `1e-12`, and assert the exact solution, `converged`, finite residual, and at most two iterations. Retain a general-Jacobian delta golden test for block-Jacobi, scalar Jacobi, and no preconditioner.

- [ ] **Step 2: Run and observe the missing generic PCG API**

Run: `cmake --build build --target testCudaLinearSolver -j2`

Expected: compilation fails because `CudaPcgSolver` is absent.

- [ ] **Step 3: Extract recurrence and adapt the general operator**

Define:

```cpp
struct CudaPcgOptions {
  int maxIterations = 250;
  double relativeTolerance = 1e-6;
  bool warmStart = true;
  int convergenceCheckInterval = 10;
};

class GTSAM_EXPORT CudaPcgSolver {
 public:
  void initialize(int dimension, const CudaPcgOptions&, cudaStream_t,
                  bool collectProfile);
  void solve(const CudaLinearOperator&, const CudaPreconditioner&,
             const double* rhs, CudaDeviceArray<double>* solution,
             cudaStream_t);
  const CudaLinearSolveStats& stats() const;
};
```

Move recurrence vectors, dot/reduction kernels, alpha/beta updates, periodic convergence synchronization, warm-start invalidation, and last-finite-iterate breakdown behavior. `CudaJacobianNormalOperator` borrows J/JT descriptors, damping, and lambda; `CudaJacobianBlockPreconditioner` moves the existing Gram-block construction/factorization and implements `CudaPreconditioner`. Keep the old `DevicePcgSolver` as a compatibility facade which wires those objects to `CudaPcgSolver`.

- [ ] **Step 4: Verify generic and general PCG regressions**

Run: `cmake --build build --target testCudaLinearSolver testCudaSparseJacobian -j2 && ./build/gtsam/linear/tests/testCudaLinearSolver && ./build/gtsam/nonlinear/tests/testCudaSparseJacobian`

Expected: generic PCG solves the 2-by-2 system and every existing general PCG delta/stat test passes.

- [ ] **Step 5: Commit**

```bash
git add gtsam/linear/cuda/CudaPcgSolver.* gtsam/nonlinear/cuda/CudaJacobianNormalOperator.* gtsam/nonlinear/cuda/DevicePcgSolver.* gtsam/linear/tests/testCudaLinearSolver.cpp gtsam/nonlinear/tests/testCudaSparseJacobian.cpp
git commit -m "refactor: separate CUDA PCG recurrence and operator"
```

## Task 7: Finish common session dispatch and statistics

**Files:**
- Modify: `gtsam/linear/cuda/CudaLinearSolver.h`
- Modify: `gtsam/linear/cuda/CudaLinearSolver.cpp`
- Modify: `gtsam/linear/tests/testCudaLinearSolver.cpp`

- [ ] **Step 1: Add dispatch lifecycle tests**

For each backend, construct `CudaLinearSolverSession`, call `analyze` once, solve two numerical systems, and assert backend, ordering flag, analysis count, factorization/solve count, PCG convergence fields, and nonnegative nonoverlapping timing fields. Assert representation mismatches fail before device allocation.

- [ ] **Step 2: Run and observe missing session solve overloads**

Run: `cmake --build build --target testCudaLinearSolver -j2`

Expected: compilation fails on the prepared-system `solve` calls.

- [ ] **Step 3: Implement tagged backend dispatch**

Give the session these explicit overloads:

```cpp
void analyze(const CudaDenseSpdSystemView&, cudaStream_t);
void analyze(const DeviceSparseSpdSystem&,
             const std::optional<std::vector<int>>&, cudaStream_t);
void analyze(int operatorDimension, cudaStream_t);
void solve(CudaDenseSpdSystemView, CudaDeviceArray<double>*, cudaStream_t);
void solve(const DeviceSparseSpdSystem&, CudaDeviceArray<double>*, cudaStream_t);
void solve(const CudaLinearOperator&, const CudaPreconditioner&, const double*,
           CudaDeviceArray<double>*, cudaStream_t);
```

Use `std::variant` for backend ownership, validate kind/options in the constructor and again at typed entry points, and aggregate backend-native stats without including frontend assembly time.

- [ ] **Step 4: Run all common backend tests**

Run: `cmake --build build --target testCudaLinearSolver -j2 && ./build/gtsam/linear/tests/testCudaLinearSolver`

Expected: all lifecycle, dispatch, stats, ordering, and backend solution tests pass.

- [ ] **Step 5: Commit**

```bash
git add gtsam/linear/cuda/CudaLinearSolver.* gtsam/linear/tests/testCudaLinearSolver.cpp
git commit -m "feat: dispatch prepared systems through shared CUDA solvers"
```

## Task 8: Refactor general CUDA LM onto the shared layer and honor ordering

**Files:**
- Modify: `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h`
- Modify: `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.cu`
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h`
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`

- [ ] **Step 1: Add trajectory, ownership, and ordering regressions**

Run the existing general fixture with cuDSS auto order, cuDSS `Ordering{...}` in a deliberately different block order, and PCG. Assert auto/user cuDSS objective/delta equivalence, `userOrderingApplied`, the exact compiled scalar vector, unchanged accepted/rejected attempt trace versus stored baseline, and unchanged PCG objective/iteration semantics. Assert dense backend and PCG-plus-ordering throw before linearization.

- [ ] **Step 2: Run and observe the old ignored-ordering behavior**

Run: `cmake --build build --target testCudaSparseLevenbergMarquardt -j2 && ./build/gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt`

Expected: the requested-order test fails because the current direct path never applies GTSAM ordering.

- [ ] **Step 3: Remove backend ownership from the producer**

Replace `solveAndEvaluate(lambda)` with explicit producer operations:

```cpp
void prepareExplicitSystem(double lambda, cudaStream_t);
CudaSparseSpdSystemView sparseSystemView();
const CudaLinearOperator& normalOperator(double lambda);
const CudaPreconditioner& preconditioner(double lambda);
void evaluateModelError(const CudaDeviceArray<double>& delta,
                        cudaStream_t);
```

The optimizer owns `CudaLinearSolverSession`, compiles `params.ordering` against `SparseJacobianColumnLayout` blocks, analyzes once per stable structure/order, restores the saved undamped diagonal for every lambda, solves, and then invokes model evaluation. Map common stats back to compatibility result fields. Preserve CPU fallback, transfer counts, attempt trace, and LM policy.

- [ ] **Step 4: Run general correctness and regression tests**

Run: `cmake --build build --target testCudaSparseJacobian testCudaSparseLevenbergMarquardt -j2 && ./build/gtsam/nonlinear/tests/testCudaSparseJacobian && ./build/gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt`

Expected: all existing and new cuDSS/PCG trajectories pass and GTSAM ordering is reported as applied.

- [ ] **Step 5: Commit**

```bash
git add gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.* gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.* gtsam/nonlinear/tests/testCudaSparse*
git commit -m "refactor: use shared CUDA linear layer in general LM"
```

## Task 9: Split persistent SFM linearization/Schur production from dense solve

**Files:**
- Create: `gtsam/slam/cuda/CudaSfmSchurProblem.h`
- Create: `gtsam/slam/cuda/CudaSfmSchurProblem.cu`
- Modify: `gtsam/slam/cuda/CudaSfmDenseSchurSolver.h`
- Modify: `gtsam/slam/cuda/CudaSfmDenseSchurSolver.cu`
- Modify: `gtsam/slam/cuda/CudaSfmProjectionLinearization.h`
- Modify: `gtsam/slam/cuda/CudaSfmProjectionLinearization.cu`
- Modify: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add persistence and dense-regression tests**

Instrument a small SFM problem with two lambda attempts. Assert projection linearization count is one, dense assembly count is two, no stale damping is present, recovered point deltas match the old fused solver, and the final trajectory/objective matches the pre-refactor golden values.

- [ ] **Step 2: Run and observe repeated projection linearization**

Run: `cmake --build build --target testCudaSfm -j2 && ./build/gtsam/slam/tests/testCudaSfm`

Expected: persistence assertion fails because `CudaSfmDenseSchurSolver::solve()` currently relinearizes per lambda.

- [ ] **Step 3: Introduce persistent SFM block storage and lifecycle**

Expose:

```cpp
class CudaSfmSchurProblem {
 public:
  void initialize(const CudaSfmProjectionBatch&, int numCameras,
                  cudaStream_t);
  void linearize(const DeviceValues&, cudaStream_t);
  CudaDenseSpdSystemView prepareDense(double lambda,
      const CudaDeviceArray<double>& damping, cudaStream_t);
  void recoverPoints(double lambda, const CudaDeviceArray<double>& damping,
      const CudaDeviceArray<double>& cameraDelta,
      CudaDeviceArray<double>* fullDelta, cudaStream_t);
};
```

Persist undamped `U` 9-by-9 camera blocks, `V` 3-by-3 point blocks, `W` 9-by-3 observation blocks, camera/point gradients, projection Jacobians/residuals, and recovery metadata. `linearize()` fills them once per outer iteration. `prepareDense()` zeroes and reconstructs exactly the current column-major dense Schur matrix and condensed RHS for each lambda. The compatibility facade calls `linearize`, shared dense solve, and `recoverPoints`.

- [ ] **Step 4: Run dense SFM and long-track tests**

Run: `cmake --build build --target testCudaSfm -j2 && ./build/gtsam/slam/tests/testCudaSfm`

Expected: one linearization is reused across lambda attempts; old dense deltas/objectives, noise handling, long-track behavior, and point recovery pass.

- [ ] **Step 5: Commit**

```bash
git add gtsam/slam/cuda/CudaSfmSchurProblem.* gtsam/slam/cuda/CudaSfmDenseSchurSolver.* gtsam/slam/cuda/CudaSfmProjectionLinearization.* gtsam/slam/tests/testCudaSfm.cpp
git commit -m "refactor: persist SFM Schur linearization across LM attempts"
```

## Task 10: Build the stable reduced-camera CSR plan and camera ordering

**Files:**
- Create: `gtsam/slam/cuda/CudaSfmReducedCsrPlan.h`
- Create: `gtsam/slam/cuda/CudaSfmReducedCsrPlan.cpp`
- Modify: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add symbolic structure and ordering tests**

Construct tracks `{c0,c1}`, `{c1,c2}`, and `{c0,c2,c3}`. Assert diagonal blocks for every camera, one upper block per co-visible pair, sorted scalar columns per row, stable repeated construction, correct camera-pair-to-81-scalar offset lookup, and no point columns. Compile `Ordering{cameraKeys[2], cameraKeys[0], cameraKeys[3], cameraKeys[1]}` to scalar ranges `{18..26,0..8,27..35,9..17}`. Reject point keys and incomplete camera orderings.

- [ ] **Step 2: Run and observe the missing reduced plan**

Run: `cmake --build build --target testCudaSfm -j2`

Expected: compilation fails on `CudaSfmReducedCsrPlan`.

- [ ] **Step 3: Implement deterministic co-visibility planning**

The plan stores `rowPointers`, `columnIndices`, diagonal scalar offsets, and a flattened lookup from `(minCamera,maxCamera,localRow,localCol)` to CSR value offset. Deduplicate camera pairs per track before insertion, always include diagonal 9-by-9 upper entries, sort scalar columns, validate `int32` limits, and expose `CudaBlockLayout cameraBlocks(cameraKeys)` plus a symbolic Gaussian graph helper used to request GTSAM COLAMD/METIS camera ordering.

- [ ] **Step 4: Run symbolic plan tests without numerical solving**

Run: `cmake --build build --target testCudaSfm -j2 && ./build/gtsam/slam/tests/testCudaSfm`

Expected: exact structure, stability, scalar expansion, and invalid-order tests pass.

- [ ] **Step 5: Commit**

```bash
git add gtsam/slam/cuda/CudaSfmReducedCsrPlan.* gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: plan sparse reduced-camera Schur systems"
```

## Task 11: Assemble and solve sparse SFM Schur systems through cuDSS

**Files:**
- Create: `gtsam/slam/cuda/CudaSfmSparseSchur.h`
- Create: `gtsam/slam/cuda/CudaSfmSparseSchur.cu`
- Modify: `gtsam/slam/cuda/CudaSfmSchurProblem.h`
- Modify: `gtsam/slam/cuda/CudaSfmSchurProblem.cu`
- Modify: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add dense-versus-sparse numerical tests**

For small unit-noise, non-unit-noise, robust-noise, long-track, and repeated-lambda cases, download dense and sparse reduced systems and compare every structurally present entry and RHS within `1e-10`. Solve both and compare camera and recovered point deltas within `1e-8`. Compare automatic and GTSAM camera ordering, requiring the ordered session flag.

- [ ] **Step 2: Run and observe the missing sparse assembly API**

Run: `cmake --build build --target testCudaSfm -j2`

Expected: compilation fails on `prepareSparse`.

- [ ] **Step 3: Implement lambda-dependent sparse Schur assembly**

Add:

```cpp
DeviceSparseSpdSystem& prepareSparse(double lambda,
    const CudaDeviceArray<double>& damping, cudaStream_t stream);
```

Upload the reduced pattern once. On every attempt: zero values/RHS, form and factor each damped 3-by-3 point block, add damped camera `U` and camera gradient, scatter `-W V^{-1} W^T` and `-W V^{-1} g_p` using plan offsets, and retain point solves for recovery. Use atomics only where multiple tracks contribute to one camera-pair entry. Analyze cuDSS once per pattern/order and solve via `CudaLinearSolverSession`.

- [ ] **Step 4: Run cross-backend SFM tests under memcheck**

Run: `cmake --build build --target testCudaSfm -j2 && compute-sanitizer --tool memcheck --error-exitcode=99 ./build/gtsam/slam/tests/testCudaSfm`

Expected: sparse and dense systems/deltas/objectives match across noise and lambda cases with no memory errors.

- [ ] **Step 5: Commit**

```bash
git add gtsam/slam/cuda/CudaSfmSparseSchur.* gtsam/slam/cuda/CudaSfmSchurProblem.* gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: solve sparse SFM Schur systems with cuDSS"
```

## Task 12: Add the implicit SFM Schur operator and camera-block PCG

**Files:**
- Create: `gtsam/slam/cuda/CudaSfmSchurOperator.h`
- Create: `gtsam/slam/cuda/CudaSfmSchurOperator.cu`
- Modify: `gtsam/slam/cuda/CudaSfmSchurProblem.h`
- Modify: `gtsam/slam/cuda/CudaSfmSchurProblem.cu`
- Modify: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add operator, preconditioner, and solve parity tests**

Apply explicit dense Schur and implicit Schur to deterministic vectors and compare within `1e-10`. Download each 9-by-9 preconditioner block and compare with the damped explicit Schur diagonal block. Solve unit, robust, long-track, and repeated-lambda fixtures using PCG; compare objective and delta against dense within configured iterative tolerance, and assert convergence/stats.

- [ ] **Step 2: Run and observe the absent operator types**

Run: `cmake --build build --target testCudaSfm -j2`

Expected: compilation fails on `CudaSfmSchurOperator`.

- [ ] **Step 3: Implement implicit application and block preconditioning**

For each apply, compute camera `U p`, per-point `q = W^T p`, solve damped 3-by-3 `V z = q`, and subtract `W z`. Build each camera diagonal as damped `U_cc - sum(W_cp V_p^-1 W_cp^T)`, factor/invert the 9-by-9 block with finite/pivot checks, and implement batched block application. Borrow persistent SFM blocks and current lambda; allocate work vectors once. Wire the operator and preconditioner into common `CudaPcgSolver`, then call the existing point recovery.

- [ ] **Step 4: Run PCG parity and sanitizer tests**

Run: `cmake --build build --target testCudaSfm -j2 && ./build/gtsam/slam/tests/testCudaSfm && compute-sanitizer --tool racecheck --error-exitcode=99 ./build/gtsam/slam/tests/testCudaSfm`

Expected: operator apply, block diagonal, converged solves, recovered points, and objectives pass; racecheck reports no hazards.

- [ ] **Step 5: Commit**

```bash
git add gtsam/slam/cuda/CudaSfmSchurOperator.* gtsam/slam/cuda/CudaSfmSchurProblem.* gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: add implicit PCG SFM Schur solver"
```

## Task 13: Route SFM full-normal through the common core

**Files:**
- Create: `gtsam/slam/cuda/CudaSfmFullNormalProblem.h`
- Create: `gtsam/slam/cuda/CudaSfmFullNormalProblem.cu`
- Modify: `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu`
- Modify: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add full-normal common-session tests**

Compare small SFM deltas/objectives for Schur dense, Schur cuDSS, Schur PCG, full-normal cuDSS, and full-normal PCG. Require full-normal cuDSS analysis reuse, no repeated damping accumulation, and correct full camera-plus-point GTSAM ordering application.

- [ ] **Step 2: Run and observe the separate direct ownership**

Run: `cmake --build build --target testCudaSfm -j2 && ./build/gtsam/slam/tests/testCudaSfm`

Expected: common stats/ordering assertions fail because the old LM path constructs `CudssSpdSolver` directly.

- [ ] **Step 3: Encapsulate the full-normal producer and use shared sessions**

Move CSR pattern ownership, numerical accumulation, saved undamped diagonal, damping restore, RHS, and the matrix-free full-normal operator into `CudaSfmFullNormalProblem`. The SFM optimizer chooses a common cuDSS or PCG session; cuDSS receives an optional full block scalar permutation, PCG receives the full-normal operator/preconditioner, and dense is rejected before setup.

- [ ] **Step 4: Run Schur/full-normal parity tests**

Run: `cmake --build build --target testCudaSfm -j2 && ./build/gtsam/slam/tests/testCudaSfm`

Expected: all five supported SFM configurations agree on the small reference objective within their direct/iterative tolerances.

- [ ] **Step 5: Commit**

```bash
git add gtsam/slam/cuda/CudaSfmFullNormalProblem.* gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu gtsam/slam/tests/testCudaSfm.cpp
git commit -m "refactor: use shared CUDA solvers for SFM full normal"
```

## Task 14: Unify SFM formulation/backend parameters and LM semantics

**Files:**
- Modify: `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h`
- Modify: `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu`
- Modify: `gtsam/slam/cuda/cuda_sfm.i`
- Modify: `gtsam/slam/tests/testCudaSfm.cpp`
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h`

- [ ] **Step 1: Add parameter and capability tests**

Assert independent `CudaSfmSystemFormulation::{Schur,FullNormal}` and `CudaLinearSolverType`; default is Schur+dense; legacy/Ceres factories map every shared LM field to the previous values; Schur supports all backends; full normal rejects dense; general rejects dense; and ordering is accepted only for cuDSS. Exercise string setters/getters and SWIG enum exposure.

- [ ] **Step 2: Run and observe coupled enum failures**

Run: `cmake --build build --target testCudaSfm testCudaSparseLevenbergMarquardt -j2`

Expected: compilation fails because current SFM parameters couple `DenseSchur` and `CudssFullNormal`.

- [ ] **Step 3: Make the parameter axes explicit**

Define:

```cpp
enum class CudaSfmSystemFormulation { Schur, FullNormal };

class CudaSfmLevenbergMarquardtParams : public LevenbergMarquardtParams {
 public:
  CudaSfmSystemFormulation formulation = CudaSfmSystemFormulation::Schur;
  CudaLinearSolverOptions linear;
  bool enableDetailedProfiling = false;
  static CudaSfmLevenbergMarquardtParams LegacyDefaults();
  static CudaSfmLevenbergMarquardtParams CeresDefaults();
};
```

Map old serialized/string spellings to the new pair for source-transition compatibility, use inherited LM lambda/termination fields throughout the SFM loop, and call capability validation before GPU allocation. Give the general params the same `CudaLinearSolverOptions` while retaining compatibility accessors for its previous enum/PCG fields.

- [ ] **Step 4: Run both optimizer suites**

Run: `cmake --build build --target testCudaSfm testCudaSparseLevenbergMarquardt -j2 && ./build/gtsam/slam/tests/testCudaSfm && ./build/gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt`

Expected: defaults reproduce prior behavior, all valid pairs solve, and every unsupported pair fails early with a precise message.

- [ ] **Step 5: Commit**

```bash
git add gtsam/slam/cuda/CudaSfmLevenbergMarquardt.* gtsam/slam/cuda/cuda_sfm.i gtsam/slam/tests/testCudaSfm.cpp gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h
git commit -m "feat: separate SFM formulation and CUDA backend"
```

## Task 15: Normalize common statistics and frontend profiles

**Files:**
- Modify: `gtsam/linear/cuda/CudaLinearSolver.h`
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h`
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp`
- Modify: `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h`
- Modify: `gtsam/slam/cuda/CudaSfmLevenbergMarquardt.cu`
- Modify: optimizer tests

- [ ] **Step 1: Add stats accounting tests**

For two outer linearizations with multiple lambda attempts, assert one analysis per stable pattern/order, one factorization per direct attempt, one solve per attempt, PCG iteration/max-hit totals, user-order flag, and that frontend assembly/transfer/model/retraction time is not included in backend analysis/factor/solve time.

- [ ] **Step 2: Run and capture missing/inconsistent fields**

Run: `cmake --build build --target testCudaLinearSolver testCudaSparseLevenbergMarquardt testCudaSfm -j2`

Expected: stats assertions fail on at least SFM, whose current fields are path-specific.

- [ ] **Step 3: Use one common stats record**

Finalize:

```cpp
struct CudaLinearSolveStats {
  CudaLinearSolverType backend;
  bool userOrderingApplied = false;
  size_t analysisCount = 0;
  size_t factorizationCount = 0;
  size_t solveCount = 0;
  size_t pcgIterationsTotal = 0;
  size_t pcgMaxIterationHits = 0;
  bool lastPcgConverged = false;
  double analysisSeconds = 0.0;
  double factorizationSeconds = 0.0;
  double solveSeconds = 0.0;
  double preconditionerSeconds = 0.0;
};
```

Store it in both result types and derive legacy counters/timing aliases from it. Keep frontend stages separately measured and document overlapping compatibility aliases.

- [ ] **Step 4: Run accounting tests**

Run: `ctest --test-dir build --output-on-failure -R 'testCudaLinearSolver|testCudaSparseLevenbergMarquardt|testCudaSfm'`

Expected: counts match actual lifecycle events and no timing double-count assertion fails.

- [ ] **Step 5: Commit**

```bash
git add gtsam/linear/cuda/CudaLinearSolver.h gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.* gtsam/slam/cuda/CudaSfmLevenbergMarquardt.* gtsam/nonlinear/tests gtsam/slam/tests
git commit -m "feat: report common CUDA linear solve statistics"
```

## Task 16: Expand benchmark matrices and machine-readable output

**Files:**
- Modify: `timing/cuda_sparse/timeCudaSparseLM.cpp`
- Modify: `timing/sfm_ba/timeCudaSFMBAL.cpp`
- Modify: `timing/sfm_ba/run_cuda_sfm_bal_benchmarks.sh`
- Create: `timing/sfm_ba/cuda_solver_matrix.md`

- [ ] **Step 1: Add CLI parsing self-checks**

Add `--list-configurations` and `--dry-run` paths. General must list `cudss-auto`, `cudss-gtsam`, `pcg`. SFM must list `schur-dense`, `schur-cudss-auto`, `schur-cudss-gtsam`, `schur-pcg`, `full-normal-cudss-auto`, `full-normal-cudss-gtsam`, and `full-normal-pcg`. Unknown pairs and order-with-non-cuDSS must return nonzero.

- [ ] **Step 2: Build and observe missing configurations**

Run: `cmake --build build --target timeCudaSparseLM timeCudaSFMBAL -j2 && ./build/timing/cuda_sparse/timeCudaSparseLM --list-configurations && ./build/timing/sfm_ba/timeCudaSFMBAL --list-configurations`

Expected: current binaries lack the new configuration list.

- [ ] **Step 3: Expose the full solver matrix and records**

Add flags `--formulation`, `--cuda-linear-solver`, `--ordering`, and `--output-format text|csv|json`. Every record includes formulation, backend, ordering, dimension/nnz, analysis/factor/solve counts, PCG iterations/convergence, initial/final objective, transfer bytes, frontend wall time, backend times, accepted iterations, and lambda attempts. The runner iterates the named matrix and fails a row whose correctness gate or finite-value check fails.

- [ ] **Step 4: Run benchmark smoke tests**

Run: `./build/timing/cuda_sparse/timeCudaSparseLM --list-configurations && ./build/timing/sfm_ba/timeCudaSFMBAL --list-configurations && ./build/timing/sfm_ba/timeCudaSFMBAL --dry-run --output-format json`

Expected: complete lists print and dry-run emits valid JSON for every supported pair.

- [ ] **Step 5: Commit**

```bash
git add timing/cuda_sparse/timeCudaSparseLM.cpp timing/sfm_ba
git commit -m "bench: compare shared CUDA solver configurations"
```

## Task 17: Document architecture, compatibility, and exact commands

**Files:**
- Create: `docs/CUDA_LINEAR_SOLVERS.md`
- Modify: relevant CUDA header comments
- Modify: top-level or timing README discovered with `rg -l 'timeCudaSFMBAL|timeCudaSparseLM' README.md docs timing`

- [ ] **Step 1: Add a documentation contract check**

Run a scriptable grep requiring the terms `prepared system`, `Schur`, `full normal`, `DenseCholesky`, `Cudss`, `Pcg`, `CUDSS_DATA_USER_PERM`, `Ordering`, `capability matrix`, and both benchmark executable names.

- [ ] **Step 2: Run the check and observe the missing document**

Run: `test -f docs/CUDA_LINEAR_SOLVERS.md && rg -q 'CUDSS_DATA_USER_PERM' docs/CUDA_LINEAR_SOLVERS.md`

Expected: exits nonzero because the document does not exist.

- [ ] **Step 3: Write operational documentation**

Explain current-to-final ownership, why formulation and backend are independent, the exact supported matrix, default behavior, GTSAM key-to-scalar expansion, cuDSS internal RHS/solution permutation, automatic versus user ordering, PCG convergence semantics, profile boundaries, compatibility names, build gates, tests, sanitizer commands, and full benchmark commands. State that general factor linearization/retract/error remain CPU-side and that SFM projection/Schur/recovery remain SFM-specific.

- [ ] **Step 4: Run the documentation contract check**

Run: `for term in 'prepared system' Schur 'full normal' DenseCholesky Cudss Pcg CUDSS_DATA_USER_PERM Ordering 'capability matrix' timeCudaSFMBAL timeCudaSparseLM; do rg -q "$term" docs/CUDA_LINEAR_SOLVERS.md || exit 1; done`

Expected: exits zero.

- [ ] **Step 5: Commit**

```bash
git add docs/CUDA_LINEAR_SOLVERS.md gtsam timing README.md
git commit -m "docs: explain shared CUDA linear solver architecture"
```

## Task 18: Run the release verification and benchmark audit

**Files:**
- Modify only files implicated by a reproduced failure.
- Record results in: `timing/sfm_ba/cuda_solver_matrix.md`

- [ ] **Step 1: Configure a clean CUDA/cuDSS release build**

Run: `cmake -S . -B build-cuda-final -DGTSAM_BUILD_WITH_CUDA=ON -DGTSAM_BUILD_WITH_CUDSS=ON -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TESTS=ON -DGTSAM_BUILD_TIMING_ALWAYS=ON`

Expected: configure succeeds and reports CUDA, cuSOLVER, cuSPARSE, and cuDSS enabled.

- [ ] **Step 2: Build all affected libraries, tests, and benchmarks**

Run: `cmake --build build-cuda-final --target gtsam testCudaLinearSolver testCudaSparseJacobian testCudaSparseLevenbergMarquardt testCudaSfm timeCudaSparseLM timeCudaSFMBAL -j2`

Expected: all targets build without warnings promoted to errors or unresolved compatibility symbols.

- [ ] **Step 3: Run the complete affected test set**

Run: `ctest --test-dir build-cuda-final --output-on-failure -R 'testCudaLinearSolver|testCudaSparseJacobian|testCudaSparseLevenbergMarquardt|testCudaSfm'`

Expected: every selected test passes.

- [ ] **Step 4: Run CUDA memory/race validation on focused boundary cases**

Run: `compute-sanitizer --tool memcheck --error-exitcode=99 ./build-cuda-final/gtsam/linear/tests/testCudaLinearSolver && compute-sanitizer --tool memcheck --error-exitcode=99 ./build-cuda-final/gtsam/slam/tests/testCudaSfm && compute-sanitizer --tool racecheck --error-exitcode=99 ./build-cuda-final/gtsam/slam/tests/testCudaSfm`

Expected: zero CUDA API, memory, leak, initialization, and race errors.

- [ ] **Step 5: Run correctness-gated benchmark matrix**

Run: `./timing/sfm_ba/run_cuda_sfm_bal_benchmarks.sh ./build-cuda-final/timing/sfm_ba/timeCudaSFMBAL && ./build-cuda-final/timing/cuda_sparse/timeCudaSparseLM --all-cuda-configurations --output-format csv`

Expected: every supported configuration reports finite output and passes its objective gate; unsupported configurations are absent.

- [ ] **Step 6: Configure and build without CUDA/cuDSS**

Run: `cmake -S . -B build-cpu-final -DGTSAM_BUILD_WITH_CUDA=OFF -DGTSAM_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release && cmake --build build-cpu-final --target gtsam -j2`

Expected: CPU-only GTSAM builds and no CUDA-only header/source is required by a CPU target.

- [ ] **Step 7: Record the measured matrix and commit the audit**

Write GPU model, CUDA/cuDSS versions, dataset/configuration, formulation/backend/ordering, dimensions/nnz, analysis/solve/PCG counts, objective, total wall time, backend wall time, and transfer bytes into `timing/sfm_ba/cuda_solver_matrix.md`.

```bash
git add timing/sfm_ba/cuda_solver_matrix.md
git commit -m "bench: record final CUDA solver matrix"
```

## Completion checklist

- [ ] Both CUDA optimizers use `gtsam/linear/cuda` sessions.
- [ ] Dense Cholesky is a shared peer of cuDSS and PCG.
- [ ] General cuDSS and PCG behavior regressions pass.
- [ ] Dense SFM behavior regression passes.
- [ ] GTSAM key ordering is expanded and passed through `CUDSS_DATA_USER_PERM`.
- [ ] SFM Schur works with dense, sparse cuDSS, and implicit PCG.
- [ ] SFM full normal works through common cuDSS and PCG.
- [ ] Repeated lambda attempts restore undamped state.
- [ ] Capability validation, common stats, benchmark records, and docs are complete.
- [ ] CUDA tests, memcheck, racecheck, benchmark gates, and CPU-only build pass.
