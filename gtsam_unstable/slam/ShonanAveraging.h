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
#include <string>

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
  explicit ShonanAveraging(const std::string& g2oFile);

  /**
   * Build graph for SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   */
  NonlinearFactorGraph buildGraphAt(size_t p) const;

  /**
   * Initialize randomly at SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   */
  Values initializeRandomlyAt(size_t p) const;

  /**
   * Calculate cost for SO(p)
   * Values should be of type SO(p)
   */
  double costAt(size_t p, const Values& values) const;

  /**
   * Try to optimize at SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   * @param initial optional initial SO(p) values
   * @return SO(p) values
   */
  Values tryOptimizingAt(
      size_t p,
      const boost::optional<const Values&> initial = boost::none) const;

  /**
   * Project from SO(p) to SO(3)
   * Values should be of type SO(p)
   */
  Values projectFrom(size_t p, const Values& values) const;

  /**
   * Calculate cost for SO(3)
   * Values should be of type SO3
   */
  double cost(const Values& values) const;

  /**
   * Optimize at different values of p until convergence.
   * @param p_max maximum value of p to try (default: 20)
   */
  void run(size_t p_max = 20) const;
};

}  // namespace gtsam
