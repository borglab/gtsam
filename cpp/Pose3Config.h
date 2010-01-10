/**
 * @file    Pose3Config.cpp
 * @brief   Configuration of 3D poses
 * @author  Frank Dellaert
 */

#pragma once

#include "Pose3.h"
#include "LieConfig.h"

namespace gtsam {

  /**
   * Pose3Config is now simply a typedef
   */
  typedef LieConfig<Pose3> Pose3Config;

  /**
   * Create a circle of n 3D poses tangent to circle of radius R, first pose at (R,0,0)
   * The convention used is the Navlab/Aerospace convention: X-forward,Y-right,Z-down
   * @param n number of poses
   * @param R radius of circle
   * @param c character to use for keys
   * @return circle of n 2D poses
   */
	Pose3Config pose3Circle(size_t n, double R, char c = 'p');

} // namespace
