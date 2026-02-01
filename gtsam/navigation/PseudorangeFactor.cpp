/**
 *  @file   PseudorangeFactor.cpp
 *  @author Sammy Guo
 *  @brief  Implementation file for GNSS Pseudorange factor
 *  @date   January 18, 2026
 **/

#include "PseudorangeFactor.h"

namespace {

/// Speed of light in a vacuum (m/s):
constexpr double CLIGHT = 299792458.0;

}  // namespace

namespace gtsam {

PseudorangeFactor::PseudorangeFactor()
    : Base(), pseudorange_(0.0), satPos_(Point3::Zero()), satClkBias_(0.0) {}

PseudorangeFactor::PseudorangeFactor(Key receiverPositionKey,
                                     Key receiverClockBiasKey,
                                     const double measuredPseudorange,
                                     const Point3& satellitePosition,
                                     const double satelliteClockBias,
                                     const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey),
      pseudorange_(measuredPseudorange),
      satPos_(satellitePosition),
      satClkBias_(satelliteClockBias) {}

//***************************************************************************
void PseudorangeFactor::print(const std::string& s,
                              const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(pseudorange_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool PseudorangeFactor::equals(const NonlinearFactor& expected,
                               double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(pseudorange_, e->pseudorange_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector PseudorangeFactor::evaluateError(
    const Point3& receiver_position, const double& receiver_clock_bias,
    OptionalMatrixType Hreceiver_pos,
    OptionalMatrixType Hreceiver_clock_bias) const {
  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  const Vector3 position_difference = receiver_position - satPos_;
  const double range = position_difference.norm();
  const double rho = range + CLIGHT * (receiver_clock_bias - satClkBias_);
  const double error = rho - pseudorange_;

  // Compute associated derivatives:
  if (Hreceiver_pos) {
    if (range < std::numeric_limits<double>::epsilon()) {
      *Hreceiver_pos = Matrix13::Zero();
    } else {
      *Hreceiver_pos = (position_difference / range).transpose();
    }
  }

  if (Hreceiver_clock_bias) {
    *Hreceiver_clock_bias = Matrix11(CLIGHT);
  }

  return Vector1(error);
}

}  // namespace gtsam
