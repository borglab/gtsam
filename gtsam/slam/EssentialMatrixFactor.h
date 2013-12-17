/*
 * @file EssentialMatrixFactor.cpp
 * @brief EssentialMatrixFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <iostream>

namespace gtsam {

/**
 * Factor that evaluates epipolar error p'Ep for given essential matrix
 */
class EssentialMatrixFactor: public NoiseModelFactor1<EssentialMatrix> {

  Point2 pA_, pB_; ///< Measurements in image A and B
  Vector vA_, vB_; ///< Homogeneous versions

  typedef NoiseModelFactor1<EssentialMatrix> Base;

public:

  /// Constructor
  EssentialMatrixFactor(Key key, const Point2& pA, const Point2& pB,
      const SharedNoiseModel& model) :
      Base(model, key), pA_(pA), pB_(pB), //
      vA_(EssentialMatrix::Homogeneous(pA)), //
      vB_(EssentialMatrix::Homogeneous(pB)) {
  }

  /// print
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor with measurements\n  ("
        << pA_.vector().transpose() << ")' and (" << pB_.vector().transpose()
        << ")'" << std::endl;
  }

  /// vector of errors returns 1D vector
  Vector evaluateError(const EssentialMatrix& E, boost::optional<Matrix&> H =
      boost::none) const {
    return (Vector(1) << E.error(vA_, vB_, H));
  }

};

} // gtsam

