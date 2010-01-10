/**
 * @file    Pose2Config.cpp
 * @brief   Configuration of 2D poses
 * @author  Frank Dellaert
 */

#pragma once

#include "Pose2.h"
#include "LieConfig.h"

namespace gtsam {

  /**
   * Pose2Config is now simply a typedef
   */
  typedef LieConfig<Pose2> Pose2Config;

  /**
   * Create a circle of n 2D poses tangent to circle of radius R, first pose at (R,0)
   * @param n number of poses
   * @param R radius of circle
   * @param c character to use for keys
   * @return circle of n 2D poses
   */
	Pose2Config pose2Circle(size_t n, double R, char c = 'p');

} // namespace
