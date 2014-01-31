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
    Vector disparities_; // measured disparities in a set of Superpixels \mathcal{V}

    typedef NoiseModelFactor1<OrientedPlane3> Base;

  public:

  // Constructor
    MultiDisparityFactor()
     {};

     /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
    MultiDisparityFactor (Key key, const Vector& disparities,
        const SharedIsotropic& noiseModel)
      : Base (noiseModel, key),
        landmarkKey_ (key),
        disparities_(disparities)
    {};


     /// print
     void print(const std::string& s="Multi-View DisaprityFactor") const;

     virtual Vector evaluateError(const OrientedPlane3& plane,
                                  boost::optional<Matrix&> H1 = boost::none) const;

};

} // gtsam
