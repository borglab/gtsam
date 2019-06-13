/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianDensity.cpp
 * @brief   A Gaussian Density
 * @author  Frank Dellaert
 * @date    Jan 21, 2012
 */

#include <gtsam/linear/GaussianDensity.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  GaussianDensity GaussianDensity::FromMeanAndStddev(Key key, const Vector& mean, const double& sigma)
  {
    return GaussianDensity(key, mean / sigma, Matrix::Identity(mean.size(), mean.size()) / sigma);
  }

  /* ************************************************************************* */
  void GaussianDensity::print(const string &s, const KeyFormatter& formatter) const
  {
    cout << s << ": density on ";
    for(const_iterator it = beginFrontals(); it != endFrontals(); ++it)
      cout << (boost::format("[%1%]")%(formatter(*it))).str() << " ";
    cout << endl;
    gtsam::print(Matrix(R()), "R: ");
    gtsam::print(Vector(d()), "d: ");
    if(model_)
      model_->print("Noise model: ");
  }

  /* ************************************************************************* */
  Vector GaussianDensity::mean() const {
    VectorValues soln = this->solve(VectorValues());
    return soln[firstFrontalKey()];
  }

  /* ************************************************************************* */
  Matrix GaussianDensity::covariance() const {
    return information().inverse();
  }

} // gtsam
