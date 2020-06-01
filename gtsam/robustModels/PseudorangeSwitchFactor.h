/**
 *  @file   PseudorangeSwitchFactor.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for Pseudorange Switchable factor
 **/

#pragma once

#include <gtsam/base/Vector.h> 
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/gnssStateVec.h>
#include <gtsam/robustModels/switchVariableLinear.h>


using namespace std;
using namespace vertigo; 

namespace gtsam {

class GTSAM_EXPORT PseudorangeSwitchFactor: public NoiseModelFactor2<gnssStateVec,SwitchVariableLinear> {

private:
  typedef NoiseModelFactor2<gnssStateVec,SwitchVariableLinear> Base;
  double measured_;
  gnssStateVec h_;

public:

  typedef boost::shared_ptr<PseudorangeSwitchFactor> shared_ptr;
  typedef PseudorangeSwitchFactor This;

  PseudorangeSwitchFactor(): measured_(0) { h_=(Matrix(1,5)<<1,1,1,1,1).finished(); }
 
  virtual ~PseudorangeSwitchFactor() {}

  PseudorangeSwitchFactor(Key j, Key k, const double deltaObs, const Matrix obsMap, const SharedNoiseModel& model):
    Base(model, j,k), measured_(deltaObs) {h_=obsMap;}

  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PseudorangeSwitchFactor(*this))); }

  /// vector of errors
  Vector evaluateError(const gnssStateVec& q,
      const SwitchVariableLinear& s,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const;

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }

}; // PseudorangeSwitchFactor Factor
} // namespace
