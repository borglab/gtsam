namespace gtsam {

// MATLAB-only declarations that are merged into the generated toolbox in
// addition to the normal GTSAM interfaces. This file exposes the native helper
// class that bridges gtsam::CustomFactor to the MATLAB callback registry.
//
// The constructor intentionally accepts gtsam::noiseModel::Base* here because
// that matches the generated MATLAB wrapper surface. The corresponding C++
// constructor takes noiseModel::Base::shared_ptr after unwrap.
#include <MatlabCustomFactor.h>
virtual class MatlabCustomFactor : gtsam::NoiseModelFactor {
  MatlabCustomFactor(gtsam::noiseModel::Base* noiseModel,
                     const gtsam::KeyVector& keys,
                     size_t callbackId);
};

}  // namespace gtsam
