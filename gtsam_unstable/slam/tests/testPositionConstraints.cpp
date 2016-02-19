/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testPositionUpperBoundX.cpp
 * @brief   Unit tests for testPositionUpperBoundX
 * @author  Krunal Chande
 * @author  Duy-Nguyen Ta
 * @author  Luca Carlone
 * @date    Dec 16, 2014
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

namespace gtsam {

template<class T>
class LinearInequalityManifold1: public NoiseModelFactor1<T> {
  typedef NoiseModelFactor1<T> Base;
  typedef LinearInequalityManifold1<T> This;
protected:
  bool active_;
public:

  /// Constructor
  LinearInequalityManifold1(Key key) :
      Base(noiseModel::Constrained::All(1), key), active_(true) {
  }

  /** implement functions needed for Testable */

  /** print */
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << " key = { " << keyFormatter(this->key()) << "}"
        << std::endl;
    if (active_)
      std::cout << " Active" << std::endl;
    else
      std::cout << " Inactive" << std::endl;
  }

  /** equals */
  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol) && (active_ == e->active_);
  }

  /** Evaluate error MUST return a *one dimensional* vector,
   * because we don't support multi-valued inequality factors
   */
  virtual Vector evaluateError(const T& x, boost::optional<Matrix&> H =
      boost::none) const = 0;

};

/**
 * This class defines an inequality upper bound on x for a Pose3
 *      x <= upperBound
 */
class PositionUpperBoundX: public LinearInequalityManifold1<Pose3> {
  double upperBound_;
  typedef LinearInequalityManifold1<Pose3> Base;
  typedef PositionUpperBoundX This;
public:
  PositionUpperBoundX(Key key, const double upperBound) :
      Base(key), upperBound_(upperBound) {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** implement functions needed for Testable */

  /** print */
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    Base::print(s);
    std::cout << "PositionUpperBoundX" << std::endl;
    std::cout << "x <= " << upperBound_ << std::endl;
  }

  /** equals */
  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol) && (upperBound_ == e->upperBound_);
  }

  /**
   * error = x - upperBound_
   */
  double computeError(const Pose3& pose) const {
    return pose.x() - upperBound_;
  }

  /**
   * error = x - upperBound_
   */
  Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H =
      boost::none) const {
    if (H)
      *H = (Matrix(1, 6) << 0, 0, 0, 1, 0, 0).finished();
    return (Vector(1) << computeError(pose)).finished();
  }
};
}

//******************************************************************************

using namespace std;
using namespace gtsam::symbol_shorthand;
using namespace gtsam;

//******************************************************************************
TEST(PositionUpperBoundX, equals ) {
  // Instantiate a class PositionUpperBoundX
  PositionUpperBoundX ineq1(X(1), 1);

  // Instantiate another class PositionUpperBoundX
  PositionUpperBoundX ineq2(X(1), 1);

  // check equals
  CHECK(ineq1.equals(ineq2, 1e-5));
}

//******************************************************************************
TEST(PositionUpperBoundX, evaluateError ) {
  // Instantiate a class PositionUpperBoundX
  PositionUpperBoundX ineq(X(1), 45.6);

  Pose3 pose(Rot3::Ypr(0.1, 0.3, 0.2), Point3(43.0, 27.8, 91.1));
  Matrix H;
  Vector error = ineq.evaluateError(pose, H);

  Matrix expectedH = numericalDerivative11<Vector, Pose3>(
          boost::bind(&PositionUpperBoundX::evaluateError, ineq, _1, boost::none),
      pose, 1e-5);

  cout << "expectedH: " << expectedH << endl;
  cout << "H: " << H << endl;

  Vector expectedError = (Vector(1) << 0.0).finished();
//  CHECK(assert_equal(expectedError, error, 1e-100));
//  CHECK(assert_equal(expectedH, H, 1e-100));

  Matrix hessian = numericalHessian<Pose3>(
          boost::bind(&PositionUpperBoundX::computeError, ineq, _1),
      pose, 1e-5);
  cout << "Hessian: \n" << hessian << endl;
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

