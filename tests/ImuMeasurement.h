#pragma once

#include <tests/Measurement.h>

namespace drs {

///
/// \brief Contains data from the IMU mesaurements.
///
class ImuMeasurement : public Measurement {
 public:
  enum Name { BODY = 0, RF_FOOT = 1, RH_FOOT = 2 };

  Name name;               ///< Unique string identifier
  Eigen::Vector3d I_a_WI;  ///< Raw acceleration from the IMU (m/s/s)
  Eigen::Vector3d I_w_WI;  ///< Raw angular velocity from the IMU (rad/s)

  virtual ~ImuMeasurement() override {}
  ImuMeasurement();
  friend std::ostream& operator<<(std::ostream& stream, const ImuMeasurement& meas);
};

}  // namespace drs
