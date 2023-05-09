/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Simulated3D.h
 * @brief  measurement functions and derivatives for simulated 3D robot
 * @author Alex Cunningham
 **/

// \callgraph

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// \namespace

namespace gtsam {
namespace simulated3D {

/**
 * This is a linear SLAM domain where both poses and landmarks are
 * 3D points, without rotation. The structure and use is based on
 * the simulated2D domain.
 */

/**
 * Prior on a single pose
 */
Point3 prior(const Point3& x, boost::optional<Matrix&> H = boost::none) {
  if (H) *H = I_3x3;
  return x;
}

/**
 * odometry between two poses
 */
Point3 odo(const Point3& x1, const Point3& x2,
    boost::optional<Matrix&> H1 = boost::none,
    boost::optional<Matrix&> H2 = boost::none) {
  if (H1) *H1 = -1 * I_3x3;
  if (H2) *H2 = I_3x3;
  return x2 - x1;
}

/**
 *  measurement between landmark and pose
 */
Point3 mea(const Point3& x, const Point3& l,
    boost::optional<Matrix&> H1 = boost::none,
    boost::optional<Matrix&> H2 = boost::none) {
  if (H1) *H1 = -1 * I_3x3;
  if (H2) *H2 = I_3x3;
  return l - x;
}

/**
 * A prior factor on a single linear robot pose
 */
struct PointPrior3D: public NoiseModelFactor1<Point3> {

  Point3 measured_; ///< The prior pose value for the variable attached to this factor

  /**
   * Constructor for a prior factor
   * @param measured is the measured/prior position for the pose
   * @param model is the measurement model for the factor (Dimension: 3)
   * @param key is the key for the pose
   */
  PointPrior3D(const Point3& measured, const SharedNoiseModel& model, Key key) :
    NoiseModelFactor1<Point3> (model, key), measured_(measured) {
  }

  /**
   * Evaluates the error at a given value of x,
   * with optional derivatives.
   * @param x is the current value of the variable
   * @param H is an optional Jacobian matrix (Dimension: 3x3)
   * @return Vector error between prior value and x (Dimension: 3)
   */
  Vector evaluateError(const Point3& x, boost::optional<Matrix&> H =
      boost::none) const override {
    return prior(x, H) - measured_;
  }
};

/**
 * Models a linear 3D measurement between 3D points
 */
struct Simulated3DMeasurement: public NoiseModelFactor2<Point3, Point3> {

  Point3 measured_; ///< Linear displacement between a pose and landmark

  /**
   * Creates a measurement factor with a given measurement
   * @param measured is the measurement, a linear displacement between poses and landmarks
   * @param model is a measurement model for the factor (Dimension: 3)
   * @param poseKey is the pose key of the robot
   * @param pointKey is the point key for the landmark
   */
  Simulated3DMeasurement(const Point3& measured, const SharedNoiseModel& model, Key i, Key j) :
        NoiseModelFactor2<Point3, Point3>(model, i, j), measured_(measured) {}

  /**
   * Error function with optional derivatives
   * @param x1 a robot pose value
   * @param x2 a landmark point value
   * @param H1 is an optional Jacobian matrix in terms of x1 (Dimension: 3x3)
   * @param H2 is an optional Jacobian matrix in terms of x2 (Dimension: 3x3)
   * @return vector error between measurement and prediction (Dimension: 3)
   */
  Vector evaluateError(const Point3& x1, const Point3& x2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const override {
    return mea(x1, x2, H1, H2) - measured_;
  }
};

}} // namespace simulated3D
