#pragma once

#include <tests/Measurement.h>

namespace gtsam {

/**
 *\brief Contains data from the IMU mesaurements.
 */
class ImuMeasurement : public Measurement {
 public:
  enum Name { BODY = 0, RF_FOOT = 1, RH_FOOT = 2 };

  Name name;               ///< Unique string identifier
  Eigen::Vector3d I_a_WI;  ///< Raw acceleration from the IMU (m/s/s)
  Eigen::Vector3d I_w_WI;  ///< Raw angular velocity from the IMU (rad/s)

  ImuMeasurement()
      : Measurement("ImuMeasurement"), I_a_WI{0, 0, 0}, I_w_WI{0, 0, 0} {}

  ~ImuMeasurement() override {}

  friend std::ostream& operator<<(std::ostream& stream,
                                  const ImuMeasurement& meas);
};

std::ostream& operator<<(std::ostream& stream, const ImuMeasurement& meas) {
  stream << "IMU Measurement at time = " << meas.time << " : \n"
         << "dt    : " << meas.dt << "\n"
         << "I_a_WI: " << meas.I_a_WI.transpose() << "\n"
         << "I_w_WI: " << meas.I_w_WI.transpose() << "\n";
  return stream;
}

}  // namespace gtsam
