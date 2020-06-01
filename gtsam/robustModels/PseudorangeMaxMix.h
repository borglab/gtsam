/**
 *  @file   PseudorangeMaxMix.h
 *  @author Ryan
 *  @brief  Header file for Pseudorange Max-Mix factor
 **/

#pragma once

#include <Eigen/Eigen>
#include <gtsam/base/Vector.h> 
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/gnssStateVec.h>

using namespace std;

namespace gtsam {

class GTSAM_EXPORT PseudorangeMaxMix: public NoiseModelFactor1<gnssStateVec> {

private:
  typedef NoiseModelFactor1<gnssStateVec> Base;
  double measured_, w_, hyp_;
  gnssStateVec h_;
  SharedNoiseModel nullHypothesisModel_;


public:

  typedef boost::shared_ptr<PseudorangeMaxMix> shared_ptr;
  typedef PseudorangeMaxMix This;

  PseudorangeMaxMix(): measured_(0) { h_=(Matrix(1,5)<<1,1,1,1,1).finished(); }
 
  virtual ~PseudorangeMaxMix() {}

  PseudorangeMaxMix(Key key, const double deltaObs, const Matrix obsMap,
    const SharedNoiseModel& model1, const SharedNoiseModel& model2,
    const double& hypNoise, double weight): NoiseModelFactor1<gnssStateVec>(model1, key), 
    measured_(deltaObs), h_(obsMap), w_(weight), hyp_(hypNoise), 
    nullHypothesisModel_(model2) {  };

  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PseudorangeMaxMix(*this))); }

  /// vector of errors
  Vector evaluateError(const gnssStateVec& q,
      boost::optional<Matrix&> H1 = boost::none) const;

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }

}; // PseudorangeMaxMix Factor
} // namespace
