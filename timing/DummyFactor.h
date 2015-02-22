/**
 * @file    DummyFactor.h
 * @brief   Just to help in timing overhead
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/slam/RegularImplicitSchurFactor.h>

namespace gtsam {

/**
 * DummyFactor
 */
template<size_t D> //
class DummyFactor: public RegularImplicitSchurFactor<D> {

public:

  typedef Eigen::Matrix<double, 2, D> Matrix2D;
  typedef std::pair<Key, Matrix2D> KeyMatrix2D;

  DummyFactor() {
  }

  DummyFactor(const std::vector<KeyMatrix2D>& Fblocks, const Matrix& E,
      const Matrix3& P, const Vector& b) :RegularImplicitSchurFactor<D>(Fblocks,E,P,b)
       {
  }

  virtual ~DummyFactor() {
  }

public:

  /**
   * @brief Dummy version to measure overhead of key access
   */
  void multiplyHessian(double alpha, const VectorValues& x,
      VectorValues& y) const {

    BOOST_FOREACH(const KeyMatrix2D& Fi, this->Fblocks_) {
      static const Vector empty;
      Key key = Fi.first;
      std::pair<VectorValues::iterator, bool> it = y.tryInsert(key, empty);
      Vector& yi = it.first->second;
      yi = x.at(key);
    }
  }

};
// DummyFactor

}

