/**
 *  @file   GnssBetweenFactor.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for GnssBetweenFactor
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point4.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>

namespace gtsam {

class GTSAM_EXPORT GnssBetweenFactor: public NoiseModelFactor2<nonBiasStates, nonBiasStates> {

private:
  typedef NoiseModelFactor2<nonBiasStates, nonBiasStates> Base;

public:

  typedef boost::shared_ptr<GnssBetweenFactor> shared_ptr;
  typedef GnssBetweenFactor This;

  GnssBetweenFactor(Key state1, Key state2, const SharedNoiseModel& model):
    Base(model, state1, state2) {;}

  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new GnssBetweenFactor(*this))); }

  /// vector of errors
  Vector evaluateError(const nonBiasStates& q, const nonBiasStates& p,
                     boost::optional<Matrix&> H1 = boost::none,
                     boost::optional<Matrix&> H2 = boost::none ) const;

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this)); }

}; // PseudorangeFactor Factor
} // namespace
