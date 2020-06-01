/**
 *  @file   PseudorangeFactor.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for Pseudorange factor
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point4.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>

namespace gtsam {

class GTSAM_EXPORT PseudorangeFactor : public NoiseModelFactor1<nonBiasStates> {

private:
typedef NoiseModelFactor1<nonBiasStates> Base;
Point3 nomXYZ_;
Point3 satXYZ_;
nonBiasStates h_;
double measured_;

public:

typedef boost::shared_ptr<PseudorangeFactor> shared_ptr;
typedef PseudorangeFactor This;

PseudorangeFactor() : measured_(0) {
        h_=(Matrix(1,5)<<1,1,1,1,1).finished();
}

virtual ~PseudorangeFactor() {
}

PseudorangeFactor(Key key, const double deltaObs, const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel& model) :
        Base(model, key), measured_(deltaObs), satXYZ_(satXYZ) {
        nomXYZ_=nomXYZ;
}


virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new PseudorangeFactor(*this)));
}

/// vector of errors
Vector evaluateError(const nonBiasStates& q,
                     boost::optional<Matrix&> H = boost::none) const;

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

}; // PseudorangeFactor Factor
} // namespace
