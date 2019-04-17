/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ShonanAveraging.h
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Shonan Averaging algorithm
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/slam/dataset.h>

#include <map>

namespace gtsam {
class NonlinearFactorGraph;

class ShonanAveraging {
 private:
  BetweenFactorPose3s factors_;
  std::map<Key, Pose3> poses_;

 public:
  /**
   * Construct from a G2O file
   */
  ShonanAveraging(const std::string& g2oFile);

  /// Build graph for SO(p)
  NonlinearFactorGraph buildGraphAt(size_t p) const;

  /// Initialize randomly at SO(p)
  Values initializeRandomlyAt(size_t p) const;

  /// Optimize
  Values optimize(const NonlinearFactorGraph& graph,
                  const Values& initial) const;

  /**
   * Calculate cost for SO(3)
   * Values should be of type SO3
   */
  double cost(const Values& values) const;

  /**
   * Try to optimize at SO(p)
   */
  Values tryOptimizingAt(size_t p) const;

  /**
   * Optimize at different values of p until convergence.
   */
  void run() const;
};

}  // namespace gtsam
