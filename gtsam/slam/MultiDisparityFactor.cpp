/*
 * MultiDisparityFactor.cpp
 *
 *  Created on: Jan 30, 2014
 *      Author: nsrinivasan7
 */


#include "MultiDisparityFactor.h"
#include <gtsam/nonlinear/NonlinearFactor.h>


using namespace std;

namespace gtsam {

//***************************************************************************

void MultiDisparityFactor::print(const string& s) const {
  cout << "Prior Factor on " << landmarkKey_ << "\n";

  for(int i = 0; i < disparities_.rows(); i++) {
    cout << "Disparity @ (" << uv_(i,0) << ", " << uv_(i,1) << ") = " << disparities_(i) << "\n";
  }

  cameraPose_.print("Camera Pose ");
  this->noiseModel_->print("  noise model: ");
  cout << "\n";
};

//***************************************************************************

Vector MultiDisparityFactor::evaluateError(const OrientedPlane3& plane,
    boost::optional<Matrix&> H) const {
  Vector e;
  e.resize(uv_.rows());
  if(H) {
    (*H).resize(uv_.rows(), 3);

    Matrix B;
    B.resize(4,3);
    B.block<3,2>(0,0) << plane.normal().basis();
    B.block<4,1>(0,2) << 0 , 0 , 0 ,1;
    B.block<1,3>(3,0) << 0 , 0 , 0;
    R(plane);

    for(int i = 0 ; i < uv_.rows() ; i++ ) {
      Matrix d = Rd_ * plane.planeCoefficients();
      (*H).row(i) = (plane.planeCoefficients().transpose() * R_.at(i) ) /( pow(d(0,0) ,2) ) * B;
    }
    e = diff(plane);
    return e;
  } else {
    R(plane); // recompute the Rd_, R_, Rn_
    e = diff(plane);
    return e;
  }
}

void MultiDisparityFactor::Rn(const OrientedPlane3& p) const {

  Rn_.resize(uv_.rows(),4);
  Matrix wRc = cameraPose_.rotation().matrix();
  Rn_.setZero();
  Rn_ << uv_ *  wRc.transpose();

  return;
}

void MultiDisparityFactor::Rd(const OrientedPlane3& p) const {

  Rd_.resize(1,4);
  Vector wTc = cameraPose_.translation().vector();

  Rd_.block<1,3>(0,0) << -1 * wTc.transpose();
  Rd_.block<1,1>(0,3) << 0.0;
  return;

}

Vector MultiDisparityFactor::diff(const OrientedPlane3& theta) const {
    Vector e;
    e.resize(uv_.rows(),1);
    Matrix wRc = cameraPose_.rotation().matrix();
    Vector wTc = cameraPose_.translation().vector();
    Vector planecoeffs = theta.planeCoefficients();
    for(int i=0; i < uv_.rows(); i++) {
      Matrix numerator = planecoeffs.block(0,0,3,1).transpose() * wRc * uv_.row(i).transpose();
      Matrix denominator = planecoeffs.block(0,0,3,1).transpose() * wTc;
      cout << "\n Plane Normals : " << planecoeffs.block(0,0,3,1);
      cout << "\nNumerator : " << numerator(0,0) << "\n Denominator : " << denominator(0,0) << "\n";
      e(i,0) =  disparities_(i,0) - ( numerator(0,0) /( denominator(0,0) + planecoeffs(0,3) ) );
      cout << e(i,0) << " = " << disparities_(i,0) << " - " << ( numerator(0,0) /( denominator(0,0) + planecoeffs(0,3) ) ) <<  "\n";
    }
    cout << "\n";
    return e;

}

}
