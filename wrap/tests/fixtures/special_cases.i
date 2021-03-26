// Check for templated class as template argument for method!
namespace gtsam {

#include <gtsam/geometry/Cal3Bundler.h>

class Cal3Bundler;

template<CALIBRATION>
class PinholeCamera {};
typedef gtsam::PinholeCamera<gtsam::Cal3Bundler> PinholeCameraCal3Bundler;

class NonlinearFactorGraph {
  template<T = {gtsam::PinholeCamera<gtsam::Cal3Bundler>}>
  void addPrior(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);
};

}