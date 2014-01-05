/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2014, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   EssentialMatrixConstraint.h
 *  @author Frank Dellaert
 *  @author Pablo Alcantarilla
 *  @date   Jan 5, 2014
 **/

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/base/Testable.h>

#include <ostream>

namespace gtsam {

/**
 * Binary factor between two Pose3 variables induced by an EssentialMatrix measurement
 * @addtogroup SLAM
 */
class EssentialMatrixConstraint: public NoiseModelFactor2<Pose3, Pose3> {

private:

  typedef EssentialMatrixConstraint This;
  typedef NoiseModelFactor2<Pose3, Pose3> Base;

  EssentialMatrix measuredE_; /** The measurement is an essential matrix */

public:

  // shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<EssentialMatrixConstraint> shared_ptr;

  /** default constructor - only use for serialization */
  EssentialMatrixConstraint() {
  }

  /**
   *  Constructor
   *  @param key1 key for first pose
   *  @param key2 key for second pose
   *  @param measuredE measured EssentialMatrix
   *  @param model noise model, 5D !
   */
  EssentialMatrixConstraint(Key key1, Key key2,
      const EssentialMatrix& measuredE, const SharedNoiseModel& model) :
      Base(model, key1, key2), measuredE_(measuredE) {
  }

  virtual ~EssentialMatrixConstraint() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** implement functions needed for Testable */

  /** print */
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "EssentialMatrixConstraint(" << keyFormatter(this->key1())
        << "," << keyFormatter(this->key2()) << ")\n";
    measuredE_.print("  measured: ");
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol)
        && this->measuredE_.equals(e->measuredE_, tol);
  }

  /** implement functions needed to derive from Factor */

  /** vector of errors */
  Vector evaluateError(const Pose3& p1, const Pose3& p2,
      boost::optional<Matrix&> Hp1 = boost::none, //
      boost::optional<Matrix&> Hp2 = boost::none) const {
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

  /** return the measured */
  const EssentialMatrix& measured() const {
    return measuredE_;
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 2;
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measuredE_);
  }
};
// \class EssentialMatrixConstraint

}/// namespace gtsam
