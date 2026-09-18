/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    PcgSolver.h
 * @brief   Device-resident preconditioned conjugate gradient (PCG) recurrence
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/LinearSolver.h>

#include <memory>

namespace gtsam::cuda {

/**
 * Preconditioned conjugate gradient (PCG) run entirely on the device.
 *
 * Solves Hx = b for symmetric positive definite H without ever seeing H: the
 * matrix arrives only as a LinearOperator computing Hp, and the preconditioner
 * only as a Preconditioner computing M⁻¹r, both supplied by the frontend. This
 * class owns the recurrence, its device vectors and scalars, and the warm-start
 * state; see PcgSolver.cu for the algorithm and the reasons it differs from the
 * textbook form.
 *
 * All work is issued on the caller's stream. The only synchronization is at a
 * convergence check, and solve() synchronizes once before returning so that
 * stats() is current. Call initialize() once per dimension, then solve()
 * repeatedly.
 */
class PcgSolver {
 public:
  /// Constructs an uninitialized solver that allocates nothing.
  PcgSolver();
  ~PcgSolver();

  PcgSolver(const PcgSolver&) = delete;
  PcgSolver& operator=(const PcgSolver&) = delete;
  PcgSolver(PcgSolver&&) noexcept;
  PcgSolver& operator=(PcgSolver&&) noexcept;

  /**
   * Allocates the workspace for a fixed system dimension and fixes the
   * convergence and warm-start policy for every later solve().
   *
   * Reinitializing discards any warm start. collectProfile adds CUDA events
   * around the convergence-check copies, which the timings in stats() need but
   * which cost a little per check.
   */
  void initialize(int dimension, const PcgOptions& options,
                  cudaStream_t stream = nullptr,
                  bool collectProfile = false);

  /**
   * Solves for x in place, starting from the previous solution when warm
   * starting is enabled and still valid, and from zero otherwise.
   *
   * rhs is device memory of the initialized dimension, and solution must
   * already be that size. Throws if the solver is uninitialized or if any
   * dimension disagrees. A solve that hits maxIterations or breaks down still
   * writes its best x and reports the outcome through stats().
   */
  void solve(const LinearOperator& linearOperator,
             const Preconditioner& preconditioner, const double* rhs,
             DeviceArray<double>* solution,
             cudaStream_t stream = nullptr);

  /// Returns cumulative counts and timings, plus the last solve's outcome.
  const LinearSolveStats& stats() const;
  /// Forces the next solve() to cold start, after the numerics of H change.
  void invalidateWarmStart();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
