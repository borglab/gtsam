/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BearingRangeFactor.h
 * @date Apr 1, 2010
 * @author Kai Ni
 * @brief a single factor contains both the bearing and the range to prevent
 * handle to pair bearing and range factors
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Testable.h>

#include <boost/concept/assert.hpp>

namespace gtsam {

// forward declaration of Bearing functor, assumed partially specified
template <typename A1, typename A2>
struct Bearing;

// forward declaration of Range functor, assumed partially specified
template <typename A1, typename A2>
struct Range;

/**
 * Binary factor for a bearing/range measurement
 * @addtogroup SLAM
 */
template <typename A1, typename A2,
          typename BEARING = typename Bearing<A1, A2>::result_type,
          typename RANGE = typename Range<A1, A2>::result_type>
class BearingRangeFactor : public NoiseModelFactor2<A1, A2> {
 public:
  typedef BearingRangeFactor<A1, A2> This;
  typedef NoiseModelFactor2<A1, A2> Base;
  typedef boost::shared_ptr<This> shared_ptr;

 private:
  // the measurement
  BEARING measuredBearing_;
  RANGE measuredRange_;

  /** concept check by type */
  BOOST_CONCEPT_ASSERT((IsTestable<BEARING>));
  BOOST_CONCEPT_ASSERT((IsTestable<RANGE>));

 public:
  BearingRangeFactor() {} /* Default constructor */
  BearingRangeFactor(Key poseKey, Key pointKey, const BEARING& measuredBearing,
                     const RANGE measuredRange, const SharedNoiseModel& model)
      : Base(model, poseKey, pointKey),
        measuredBearing_(measuredBearing),
        measuredRange_(measuredRange) {}

  virtual ~BearingRangeFactor() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** Print */
  virtual void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "BearingRangeFactor(" << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ")\n";
    measuredBearing_.print("measured bearing: ");
    std::cout << "measured range: " << measuredRange_ << std::endl;
    this->noiseModel_->print("noise model:\n");
  }

  /** equals */
  virtual bool equals(const NonlinearFactor& expected, RANGE tol = 1e-9) const {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol) &&
           fabs(this->measuredRange_ - e->measuredRange_) < tol &&
           this->measuredBearing_.equals(e->measuredBearing_, tol);
  }

  /** h(x)-z -> between(z,h(x)) for BEARING manifold */
  Vector evaluateError(const A1& pose, const A2& point,
                       boost::optional<Matrix&> H1,
                       boost::optional<Matrix&> H2) const {
    typename MakeJacobian<BEARING, A1>::type HB1;
    typename MakeJacobian<BEARING, A2>::type HB2;
    typename MakeJacobian<RANGE, A1>::type HR1;
    typename MakeJacobian<RANGE, A2>::type HR2;

    BEARING y1 = Bearing<A1, A2>()(pose, point, H1 ? &HB1 : 0, H2 ? &HB2 : 0);
    Vector e1 = traits<BEARING>::Logmap(traits<BEARING>::Between(measuredBearing_,y1));

    RANGE y2 = Range<A1, A2>()(pose, point, H1 ? &HR1 : 0, H2 ? &HR2 : 0);
    Vector e2 = traits<RANGE>::Logmap(traits<RANGE>::Between(measuredRange_, y2));

    if (H1) {
      H1->resize(HB1.RowsAtCompileTime + HR1.RowsAtCompileTime, HB1.ColsAtCompileTime);
      *H1 << HB1, HR1;
    }
    if (H2) {
      H2->resize(HB2.RowsAtCompileTime + HR2.RowsAtCompileTime, HB2.ColsAtCompileTime);
      *H2 << HB2, HR2;
    }
    return concatVectors(2, &e1, &e2);
  }

  /** return the measured */
  const std::pair<BEARING, RANGE> measured() const {
    return std::make_pair(measuredBearing_, measuredRange_);
  }

 private:
  /** Serialization function */
  friend typename boost::serialization::access;
  template <typename ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(measuredBearing_);
    ar& BOOST_SERIALIZATION_NVP(measuredRange_);
  }
};  // BearingRangeFactor

}  // namespace gtsam
