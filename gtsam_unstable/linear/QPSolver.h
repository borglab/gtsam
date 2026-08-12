/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPSolver.h
 * @brief    Compatibility wrapper for the constrained active-set QP solver.
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     6/16/16
 */

#pragma once

#include <gtsam/config.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam/linear/VectorValues.h>
#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/linear/InfeasibleInitialValues.h>
#include <gtsam_unstable/linear/InfeasibleOrUnboundedProblem.h>
#include <gtsam_unstable/linear/QP.h>

#include <utility>

namespace gtsam {

/**
 * Legacy unstable QP solver compatibility wrapper.
 *
 * @deprecated Use gtsam::QpProblem with
 * gtsam::ActiveSetSolver instead.
 */
class GTSAM_UNSTABLE_EXPORT QPSolver {
 public:
  /** Construct from a legacy unstable QP problem. */
  explicit QPSolver(const QP& qp);

  /** Optimize from caller-provided feasible initial values. */
  std::pair<VectorValues, VectorValues> optimize(
      const VectorValues& initialValues,
      const VectorValues& duals = VectorValues(),
      bool useWarmStart = false) const;

  /** Find a feasible vector-valued point and optimize. */
  std::pair<VectorValues, VectorValues> optimize() const;

 private:
  const QP& qp_;
};

}  // namespace gtsam

#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43
