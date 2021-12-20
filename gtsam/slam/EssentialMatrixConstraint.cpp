/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2014, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   EssentialMatrixConstraint.cpp
 *  @author Frank Dellaert
 *  @author Pablo Alcantarilla
 *  @date   Jan 5, 2014
 **/

#include <gtsam/slam/EssentialMatrixConstraint.h>
//#include <gtsam/linear/GaussianFactor.h>
//#include <gtsam/base/Testable.h>

#include <ostream>

namespace gtsam {

/* ************************************************************************* */
void EssentialMatrixConstraint::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  std::cout << s << "EssentialMatrixConstraint(" << keyFormatter(this->key1())
      << "," << keyFormatter(this->key2()) << ")\n";
  measuredE_.print("  measured: ");
  this->noiseModel_->print("  noise model: ");
}

/* ************************************************************************* */
bool EssentialMatrixConstraint::equals(const NonlinearFactor& expected,
    double tol) const {
  const This *e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol)
      && this->measuredE_.equals(e->measuredE_, tol);
}

/* ************************************************************************* */
Vector EssentialMatrixConstraint::evaluateError(const Pose3& p1,
    const Pose3& p2, boost::optional<Matrix&> Hp1,
    boost::optional<Matrix&> Hp2) const {

  // compute relative Pose3 between p1 and p2
  Pose3 _1P2_ = p1.between(p2, Hp1, Hp2);

  // convert to EssentialMatrix
  Matrix D_hx_1P2;
  EssentialMatrix hx = EssentialMatrix::FromPose3(_1P2_,
      (Hp1 || Hp2) ? boost::optional<Matrix&>(D_hx_1P2) : boost::none);

  // Calculate derivatives if needed
  if (Hp1) {
    // Hp1 will already contain the 6*6 derivative D_1P2_p1
    const Matrix& D_1P2_p1 = *Hp1;
    // The 5*6 derivative is obtained by chaining with 5*6 D_hx_1P2:
    *Hp1 = D_hx_1P2 * D_1P2_p1;
  }
  if (Hp2) {
    // Hp2 will already contain the 6*6 derivative D_1P2_p1
    const Matrix& D_1P2_p2 = *Hp2;
    // The 5*6 derivative is obtained by chaining with 5*6 D_hx_1P2:
    *Hp2 = D_hx_1P2 * D_1P2_p2;
  }

  // manifold equivalent of h(x)-z -> log(z,h(x))
  return measuredE_.localCoordinates(hx); // 5D error
}

} /// namespace gtsam
