// Check for templated class as template argument for method!
namespace gtsam {

#include <gtsam/geometry/Cal3Bundler.h>

class Cal3Bundler;

template<CALIBRATION>
class PinholeCamera {};
typedef gtsam::PinholeCamera<gtsam::Cal3Bundler> PinholeCameraCal3Bundler;

class NonlinearFactorGraph {
  template <T = {gtsam::PinholeCamera<gtsam::Cal3Bundler>}>
  void addPrior(size_t key, const T& prior,
                const gtsam::noiseModel::Base* noiseModel);
};

// Typedef with template as template arg.
template<CALIBRATION, POINT>
class GeneralSFMFactor {};
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3> GeneralSFMFactorCal3Bundler;

// Template as template arg for class property.
class SfmTrack {
  std::vector<std::pair<size_t, gtsam::Point2>> measurements;
};

}  // namespace gtsam


// class VariableIndex {
// VariableIndex();
//   // template<T = {gtsam::SymbolicFactorGraph, gtsam::GaussianFactorGraph, gtsam::NonlinearFactorGraph, gtsam::FactorGraph}>
//   VariableIndex(const T& graph);
//   VariableIndex(const T& graph, size_t nVariables);
// };
