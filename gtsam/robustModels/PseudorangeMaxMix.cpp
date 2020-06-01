/**
 *  @file   PseudorangeMaxMix.cpp
 *  @author Ryan
 *  @brief  Implementation file for pseudorange max-mix factor
 **/

#include "PseudorangeMaxMix.h"

using namespace std;

namespace gtsam {

//***************************************************************************
  Vector PseudorangeMaxMix::evaluateError(const gnssStateVec& q,
    boost::optional<Matrix&> H1 ) const {

    auto error = ( Vector(1) << h_.transpose()*q-measured_ ).finished();

    auto g1 = noiseModel::Gaussian::Covariance(
      (( Vector(1) << hyp_).finished()).asDiagonal());
    auto g2 = noiseModel::Gaussian::Covariance(
      (( Vector(1) << hyp_/w_).finished()).asDiagonal());

    double m1 = this->noiseModel()->distance(error);
    Matrix info1(g1->information());
    double nu1 = 1.0/sqrt(inverse(info1).determinant());
    double l1 = nu1 * exp(-0.5*m1);

    double m2 = nullHypothesisModel_->distance(error);
    Matrix info2(g2->information());
    double nu2 = 1.0/sqrt(inverse(info2).determinant());
    double l2 = nu2 * exp(-0.5*m2);

    if (H1) { (*H1) = (Matrix(1,5) << h_.transpose() ).finished(); }

    if (l2>l1) {
      if (H1) *H1 = *H1 * w_;
      error *= sqrt(w_);
    }

    return (Vector(1) << error).finished();
  }
} // namespace
