/* ----------------------------------------------------------------------------

 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file MultiDisparityFactor.h
 * @date Jan 30, 2013
 * @author Natesh Srinivasan
 * @brief A factor for modeling the disparity across multiple views
 */

#include <gtsam/geometry/OrientedPlane3.h>

namespace gtsam {

/**
 * Unary factor on measured disparity from multiple views as deterministic function of camera pose
 */

class MutiDisparityFactor : public NoiseModelFactor1<OrientedPlane3> {

protected :
  Symbol landmarkSymbol_;
  OrientedPlane3 measured_p_;

  typedef NoiseModelFactor1<OrientedPlane3 > Base;

public:

  /// Constructor
  MutiDisparityFactor ()
   {}

   /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
  MutiDisparityFactor (const Vector&z, const SharedGaussian& noiseModel,
                const Symbol& pose,
                const Symbol& landmark)
     : Base (noiseModel, landmark),
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

class OrientedPlane3DirectionPrior: public NoiseModelFactor1<OrientedPlane3> {

 protected:
   OrientedPlane3 measured_p_; /// measured plane parameters
   Symbol landmarkSymbol_;

   typedef NoiseModelFactor1<OrientedPlane3 > Base;

 public:

   typedef OrientedPlane3DirectionPrior This;
   /// Constructor
   OrientedPlane3DirectionPrior ()
   {}

   /// Constructor with measured plane coefficients (a,b,c,d), noise model, landmark symbol
   OrientedPlane3DirectionPrior (const Symbol& landmark, const Vector&z,
       const SharedGaussian& noiseModel)
     : Base (noiseModel, landmark),
       landmarkSymbol_ (landmark)
   {
     measured_p_ = OrientedPlane3 (Sphere2 (z (0), z (1), z (2)), z (3));
   }

   /// print
   void print(const std::string& s) const;

   /** equals */
   virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

   virtual Vector evaluateError(const OrientedPlane3& plane,
                                boost::optional<Matrix&> H = boost::none) const;
};


};
}
