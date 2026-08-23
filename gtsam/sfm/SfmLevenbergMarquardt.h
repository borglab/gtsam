/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmLevenbergMarquardt.h
 * @brief CPU Levenberg-Marquardt optimizer with optional landmark elimination.
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/sfm/SfmEliminationMode.h>

#include <memory>
#include <string>

namespace gtsam {

class SfmLevenbergMarquardtOptimizer;

/// Parameters for CPU SFM Levenberg-Marquardt optimization.
class GTSAM_EXPORT SfmLevenbergMarquardtParams
    : public LevenbergMarquardtParams {
 public:
  using OptimizerType = SfmLevenbergMarquardtOptimizer;

  /// Point-elimination policy. CPU SFM optimization defaults to Full.
  SfmEliminationMode eliminationMode = SfmEliminationMode::Full;

  /// Construct the fastest measured CPU defaults: Full + MultifrontalSolver.
  SfmLevenbergMarquardtParams();

  /// Return parameters initialized with GTSAM's legacy LM defaults.
  static SfmLevenbergMarquardtParams legacyDefaults();
  /// Return parameters initialized with GTSAM's Ceres-style LM defaults.
  static SfmLevenbergMarquardtParams ceresDefaults();

  SfmEliminationMode getEliminationMode() const { return eliminationMode; }
  void setEliminationMode(SfmEliminationMode mode) { eliminationMode = mode; }

  void print(const std::string& str = "") const override;
  bool equals(const SfmLevenbergMarquardtParams& other,
              double tol = 1e-9) const;

  /// Return a polymorphic copy preserving the SFM elimination mode.
  std::shared_ptr<NonlinearOptimizerParams> clone() const {
    return std::make_shared<SfmLevenbergMarquardtParams>(*this);
  }
};

/**
 * CPU SFM optimizer supporting either a joint solve or Schur elimination of
 * Point3 and Unit3 variables. `MULTIFRONTAL_SOLVER` performs Schur elimination
 * in one fused factorization; other solvers receive an explicitly reduced
 * graph containing every other variable type.
 */
class GTSAM_EXPORT SfmLevenbergMarquardtOptimizer
    : public LevenbergMarquardtOptimizer {
 public:
  /**
   * Create a fill-reducing ordering for the reduced system. Active Point3 and
   * Unit3 variables are symbolically eliminated in natural key order before
   * METIS orders every remaining variable. This is the ordering accepted by
   * Schur-mode parameters.
   */
  static Ordering CreateReducedOrdering(const NonlinearFactorGraph& graph,
                                        const Values& initialValues);

  /**
   * Create a Schur ordering by prefixing a reduced-system ordering with all
   * other active graph keys in natural key order. This complete ordering is
   * used by Full mode and by the fused Schur multifrontal path.
   */
  static Ordering CreateSchurOrdering(const NonlinearFactorGraph& graph,
                                      const Ordering& reducedOrdering);

  SfmLevenbergMarquardtOptimizer(const NonlinearFactorGraph& graph,
                                 const Values& initialValues,
                                 const SfmLevenbergMarquardtParams& params =
                                     SfmLevenbergMarquardtParams());
  ~SfmLevenbergMarquardtOptimizer() override;

  const SfmLevenbergMarquardtParams& sfmParams() const { return sfmParams_; }

 protected:
  VectorValues solve(const GaussianFactorGraph& graph,
                     const NonlinearOptimizerParams& params) const override;

  bool ensureMultifrontalSolver(const NonlinearOptimizerParams& params,
                                const Values& values) const override;

 private:
  struct SchurState;

  SfmLevenbergMarquardtParams sfmParams_;
  mutable std::unique_ptr<SchurState> schurState_;
};

}  // namespace gtsam
