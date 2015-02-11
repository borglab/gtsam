/*
 * @file OrientedPlane3Factor.cpp
 * @brief OrientedPlane3 Factor class
 * @author Alex Trevor
 * @date December 22, 2013
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/inference/Key.h>
#include <iostream>

namespace gtsam {

/**
 * Factor to measure a planar landmark from a given pose
 */
  class OrientedPlane3Factor: public NoiseModelFactor2<Pose3, OrientedPlane3> {

    protected:
      Symbol poseSymbol_;
      Symbol landmarkSymbol_;
      Vector3 measured_coeffs_;
      OrientedPlane3 measured_p_;
      
      typedef NoiseModelFactor2<Pose3, OrientedPlane3 > Base;

    public:
      
      /// Constructor
      OrientedPlane3Factor ()
      {}
      
      /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
      OrientedPlane3Factor (const Vector&z, const SharedGaussian& noiseModel,
                   const Symbol& pose,
                   const Symbol& landmark)
        : Base (noiseModel, pose, landmark),
          poseSymbol_ (pose),
          landmarkSymbol_ (landmark),
          measured_coeffs_ (z)
      {
        measured_p_ = OrientedPlane3 (Unit3 (z (0), z (1), z (2)), z (3));
      }

      /// print
      void print(const std::string& s="PlaneFactor") const;
      
      virtual Vector evaluateError(const Pose3& pose, const OrientedPlane3& plane,
                                   boost::optional<Matrix&> H1 = boost::none,
                                   boost::optional<Matrix&> H2 = boost::none) const 
      {
        OrientedPlane3 predicted_plane = OrientedPlane3::Transform (plane, pose, H1, H2);
        Vector err;
        err << predicted_plane.error (measured_p_);
        return (err);
      };
  };

  // TODO: Convert this factor to dimension two, three dimensions is redundant for direction prior
  class OrientedPlane3DirectionPrior: public NoiseModelFactor1<OrientedPlane3> {

    protected:
      OrientedPlane3 measured_p_; /// measured plane parameters
      Key landmarkKey_;

      typedef NoiseModelFactor1<OrientedPlane3 > Base;

    public:

      typedef OrientedPlane3DirectionPrior This;
      /// Constructor
      OrientedPlane3DirectionPrior ()
      {}

      /// Constructor with measured plane coefficients (a,b,c,d), noise model, landmark symbol
      OrientedPlane3DirectionPrior (Key key, const Vector&z,
          const SharedGaussian& noiseModel)
        : Base (noiseModel, key),
          landmarkKey_ (key)
      {
        measured_p_ = OrientedPlane3 (Unit3 (z (0), z (1), z (2)), z (3));
      }

      /// print
      void print(const std::string& s) const;

      /** equals */
      virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

      virtual Vector evaluateError(const OrientedPlane3& plane,
                                   boost::optional<Matrix&> H = boost::none) const;
  };

} // gtsam

