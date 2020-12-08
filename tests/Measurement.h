#pragma once

#include <Eigen/Core>
#include <string>

namespace drs {

///
/// \brief This is the base class for all measurement types.
///
class Measurement {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t dt;      ///< Time since the last message of this type (nanoseconds).
  uint64_t time;    ///< ROS time message recieved (nanoseconds).
  std::string type; ///< The type of message (to enable dynamic/static casting).

  virtual ~Measurement() {}
  Measurement();
  Measurement(std::string _type);
};

} // namespace drs
