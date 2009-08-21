// Christian Potthast
// Simulated2D Odometry

#include "NonlinearFactor.h"
#include "simulated2D.h"

namespace gtsam {

  struct Simulated2DOdometry : public NonlinearFactor2 {
  Simulated2DOdometry(const Vector& z, double sigma, const std::string& key1, const std::string& key2):
    NonlinearFactor2(z, sigma, odo, key1, Dodo1, key2, Dodo2) {}
  };

} // namespace gtsam
