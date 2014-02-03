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
    gtsam::Pose3 cameraPose_; // not a random variable , treated as a parameter to the factor
    Vector disparities_; // measured disparity at a Pixel (u,v)
    Eigen::Matrix<double,Eigen::Dynamic,3> uv_; // the 2D image coordinates. It is assumed here that the image co-ordinates are
    // aligned with the disparity

    mutable Eigen::MatrixXd  Rd_; // the denominator matrix
    mutable Eigen::Matrix<double, Eigen::Dynamic, 3> Rn_; // the numerator matrix
    mutable std::vector<Eigen::Matrix<double,3,3> > R_;

    typedef NoiseModelFactor1<OrientedPlane3> Base;

  public:

  // Constructor
    MultiDisparityFactor()
     {};

     /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
    MultiDisparityFactor (Key key, const Vector& disparities, const Eigen::Matrix<double,Eigen::Dynamic,3>& uv,
        const gtsam::Pose3& cameraPose,const SharedIsotropic& noiseModel)
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


     void Rn(const OrientedPlane3& p) const;
     inline const Eigen::MatrixXd Rn() {
       return Rn_;
     }

     void Rd(const OrientedPlane3& p) const;
     inline const Eigen::MatrixXd Rd() {
       return Rd_;
     }

     void R(const OrientedPlane3& p) const {
       Rd(p);
       Rn(p);
       for(int i =0; i < Rn_.rows(); i++) {
         Matrix Rnr = Rn_.row(i);
         R_.push_back( Rd_.transpose() * Rnr - Rnr.transpose() * Rd_ );
       }
     }


     inline const Eigen::Matrix<double,3,3> getR(int i) {
       return R_.at(i);
     }

     bool equals(const NonlinearFactor &p, double tol = 1e-9) const  {

     }

     // compute the differene between predivted and actual disparity
     Vector diff(const OrientedPlane3& theta) const;
};

} // gtsam
