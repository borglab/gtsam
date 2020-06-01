/**
 *  @file   PhaseFactor.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for Carrier-Phase Factor
 **/

#pragma once
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>

namespace gtsam {

class GTSAM_EXPORT PhaseFactor : public NoiseModelFactor2<nonBiasStates, phaseBias> {

private:
typedef NoiseModelFactor2<nonBiasStates, phaseBias> Base;
Point3 satXYZ_;
Point3 nomXYZ_;
double measured_;
nonBiasStates h_;

public:

typedef boost::shared_ptr<PhaseFactor> shared_ptr;
typedef PhaseFactor This;

PhaseFactor() : measured_(0) {
        h_=(Matrix(1,5)<<1,1,1,1,1).finished();
}

virtual ~PhaseFactor() {
}

PhaseFactor(Key deltaStates, Key bias, const double measurement,
            const Point3 satXYZ, const Point3 nomXYZ,const SharedNoiseModel &model) :
        Base(model, deltaStates, bias), measured_(measurement)
{
        satXYZ_=satXYZ;
        nomXYZ_=nomXYZ;
}

virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new PhaseFactor(*this)));
}

Vector evaluateError(const nonBiasStates& q, const phaseBias& g,
                     boost::optional<Matrix&> H1 = boost::none,
                     boost::optional<Matrix&> H2 = boost::none ) const;

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

}; // PhaseFactor Factor
} // namespace
