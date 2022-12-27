/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  TSAMFactors.h
 *  @brief TSAM 1 Factors, simpler than the hierarchical TSAM 2
 *  @author Frank Dellaert
 *  @date May 2014
 */

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * DeltaFactor: relative 2D measurement between Pose2 and Point2
 */
class DeltaFactor: public NoiseModelFactorN<Pose2, Point2> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(DeltaFactor, 2);

public:
  typedef DeltaFactor This;
  typedef NoiseModelFactorN<Pose2, Point2> Base;
  typedef boost::shared_ptr<This> shared_ptr;

private:
  Point2 measured_; ///< the measurement

public:

  /// Constructor
  DeltaFactor(Key i, Key j, const Point2& measured,
      const SharedNoiseModel& model) :
      Base(model, i, j), measured_(measured) {
  }

  /// Evaluate measurement error h(x)-z
  Vector evaluateError(const Pose2& pose, const Point2& point,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const override {
    return pose.transformTo(point, H1, H2) - measured_;
  }
};

/**
 * DeltaFactorBase: relative 2D measurement between Pose2 and Point2, with Basenodes
 */
class DeltaFactorBase: public NoiseModelFactorN<Pose2, Pose2, Pose2, Point2> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(DeltaFactorBase, 4);

public:
  typedef DeltaFactorBase This;
  typedef NoiseModelFactorN<Pose2, Pose2, Pose2, Point2> Base;
  typedef boost::shared_ptr<This> shared_ptr;

private:
  Point2 measured_; ///< the measurement

public:

  /// Constructor
  DeltaFactorBase(Key b1, Key i, Key b2, Key j, const Point2& measured,
      const SharedNoiseModel& model) :
      Base(model, b1, i, b2, j), measured_(measured) {
  }

  /// Evaluate measurement error h(x)-z
  Vector evaluateError(const Pose2& base1, const Pose2& pose,
      const Pose2& base2, const Point2& point, //
      boost::optional<Matrix&> H1 = boost::none, //
      boost::optional<Matrix&> H2 = boost::none, //
      boost::optional<Matrix&> H3 = boost::none, //
      boost::optional<Matrix&> H4 = boost::none) const override {
    if (H1 || H2 || H3 || H4) {
      // TODO use fixed-size matrices
      Matrix D_pose_g_base1, D_pose_g_pose;
      Pose2 pose_g = base1.compose(pose, D_pose_g_base1, D_pose_g_pose);
      Matrix D_point_g_base2, D_point_g_point;
      Point2 point_g = base2.transformFrom(point, D_point_g_base2,
          D_point_g_point);
      Matrix D_e_pose_g, D_e_point_g;
      Point2 d = pose_g.transformTo(point_g, D_e_pose_g, D_e_point_g);
      if (H1)
        *H1 = D_e_pose_g * D_pose_g_base1;
      if (H2)
        *H2 = D_e_pose_g * D_pose_g_pose;
      if (H3)
        *H3 = D_e_point_g * D_point_g_base2;
      if (H4)
        *H4 = D_e_point_g * D_point_g_point;
      return d - measured_;
    } else {
      Pose2 pose_g = base1.compose(pose);
      Point2 point_g = base2.transformFrom(point);
      Point2 d = pose_g.transformTo(point_g);
      return d - measured_;
    }
  }
};

/**
 * OdometryFactorBase: Pose2 odometry, with Basenodes
 */
class OdometryFactorBase: public NoiseModelFactorN<Pose2, Pose2, Pose2, Pose2> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(OdometryFactorBase, 4);

public:
  typedef OdometryFactorBase This;
  typedef NoiseModelFactorN<Pose2, Pose2, Pose2, Pose2> Base;
  typedef boost::shared_ptr<This> shared_ptr;

private:
  Pose2 measured_; ///< the measurement

public:

  /// Constructor
  OdometryFactorBase(Key b1, Key i, Key b2, Key j, const Pose2& measured,
      const SharedNoiseModel& model) :
      Base(model, b1, i, b2, j), measured_(measured) {
  }

  /// Evaluate measurement error h(x)-z
  Vector evaluateError(const Pose2& base1, const Pose2& pose1,
      const Pose2& base2, const Pose2& pose2, //
      boost::optional<Matrix&> H1 = boost::none, //
      boost::optional<Matrix&> H2 = boost::none, //
      boost::optional<Matrix&> H3 = boost::none, //
      boost::optional<Matrix&> H4 = boost::none) const override {
    if (H1 || H2 || H3 || H4) {
      // TODO use fixed-size matrices
      Matrix D_pose1_g_base1, D_pose1_g_pose1;
      Pose2 pose1_g = base1.compose(pose1, D_pose1_g_base1, D_pose1_g_pose1);
      Matrix D_pose2_g_base2, D_pose2_g_pose2;
      Pose2 pose2_g = base2.compose(pose2, D_pose2_g_base2, D_pose2_g_pose2);
      Matrix D_e_pose1_g, D_e_pose2_g;
      Pose2 d = pose1_g.between(pose2_g, D_e_pose1_g, D_e_pose2_g);
      if (H1)
        *H1 = D_e_pose1_g * D_pose1_g_base1;
      if (H2)
        *H2 = D_e_pose1_g * D_pose1_g_pose1;
      if (H3)
        *H3 = D_e_pose2_g * D_pose2_g_base2;
      if (H4)
        *H4 = D_e_pose2_g * D_pose2_g_pose2;
      return measured_.localCoordinates(d);
    } else {
      Pose2 pose1_g = base1.compose(pose1);
      Pose2 pose2_g = base2.compose(pose2);
      Pose2 d = pose1_g.between(pose2_g);
      return measured_.localCoordinates(d);
    }
  }
};

}

