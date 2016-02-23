/**
 * @file Predictor.h
 * @brief Dynamics Predictor
 * @author Paul Drews
 */

#pragma once

#include <gtsam_unstable/base/dllexport.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/ProductLieGroup.h>

namespace gtsam {

struct Predictor {
  Predictor(Matrix31 x,double dt);
  Matrix3 operator()(Matrix51 theta, OptionalJacobian<3,5> H);
private:
  //std:vector<RBF> rbf;
  Matrix<3,5> dt_rbf_;
};

} // \namespace gtsam
