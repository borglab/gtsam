// Christian Potthast
// Simulated2D Odometry

#include "NonlinearFactor.h"
#include "simulated2D.h"

namespace gtsam {

  struct Simulated2DMeasurement : public NonlinearFactor2 {
  Simulated2DMeasurement(const Vector& z, double sigma, const std::string& key1, const std::string& key2):
    NonlinearFactor2(z, sigma, mea, key1, Dmea1, key2, Dmea2) {}
  };

} // namespace gtsam
