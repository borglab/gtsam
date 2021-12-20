#pragma once

#include <Eigen/Core>
#include <string>

namespace gtsam {

/**
 * \brief This is the base class for all measurement types.
 */
class Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  size_t dt;    ///< Time since the last message of this type (nanoseconds).
  size_t time;  ///< ROS time message recieved (nanoseconds).
  ///< The type of message (to enable dynamic/static casting).
  std::string type;

  Measurement() : dt(0), time(0), type("UNDEFINED") {}
  Measurement(std::string _type) : dt(0), time(0), type(_type) {}

  virtual ~Measurement() {}
};

}  // namespace gtsam
