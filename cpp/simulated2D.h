/**
 * @file    simulated2D.h
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "NonlinearFactor.h"

// \namespace

namespace gtsam {

  /**
   * Prior on a single pose
   */
  Vector prior (const Vector& x);
  Matrix Dprior(const Vector& x);

  /**
   * odometry between two poses
   */
  Vector odo(const Vector& x1, const Vector& x2);
  Matrix Dodo1(const Vector& x1, const Vector& x2);
  Matrix Dodo2(const Vector& x1, const Vector& x2);

  /**
   *  measurement between landmark and pose
   */
  Vector mea(const Vector& x,  const Vector& l);
  Matrix Dmea1(const Vector& x, const Vector& l);
  Matrix Dmea2(const Vector& x, const Vector& l);

} // namespace gtsam
