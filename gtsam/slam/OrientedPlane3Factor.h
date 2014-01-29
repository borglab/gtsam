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
        measured_p_ = OrientedPlane3 (Sphere2 (z (0), z (1), z (2)), z (3));
      }

      /// print
      void print(const std::string& s="PlaneFactor") const;
      
      virtual Vector evaluateError(const Pose3& pose, const OrientedPlane3& plane, 
                                   boost::optional<Matrix&> H1 = boost::none,
                                   boost::optional<Matrix&> H2 = boost::none) const 
      {
        OrientedPlane3 predicted_plane = OrientedPlane3::Transform (plane, pose, H1, H2);
        Vector error = predicted_plane.error (measured_p_);
        return (error);
      };
  };

} // gtsam

