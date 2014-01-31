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

#pragma once

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>


namespace gtsam {

/**
 * Unary factor on measured disparity from multiple views as deterministic function of camera pose
 */

class MultiDisparityFactor: public NoiseModelFactor1<OrientedPlane3> {

  protected :

    Key landmarkKey_; // the key of the hidden plane in the world
    Pose3 cameraPose_; // not a random variable , treated as a parameter to the factor
    Vector disparities_; // measured disparity at a Pixel (u,v)
    Eigen::Matrix<int,Eigen::Dynamic,2> uv_; // the 2D image coordinates. It is assumed here that the image co-ordinates are
    // aligned with the disparity

    typedef NoiseModelFactor1<OrientedPlane3> Base;

  public:

  // Constructor
    MultiDisparityFactor()
     {};

     /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
    MultiDisparityFactor (Key key, const Vector& disparities, const Eigen::Matrix<int,Eigen::Dynamic,2>& uv,
        const Pose3& cameraPose, const SharedIsotropic& noiseModel)
      : Base (noiseModel, key),
        landmarkKey_ (key),
        disparities_(disparities),
        uv_(uv),
        cameraPose_(cameraPose)
    {};


     /// print
     void print(const std::string& s="Multi-View DisaprityFactor") const;

     virtual Vector evaluateError(const OrientedPlane3& plane,
                                  boost::optional<Matrix&> H1 = boost::none) const;

};

} // gtsam
