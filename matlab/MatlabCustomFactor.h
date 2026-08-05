#pragma once

#include <gtsam/nonlinear/CustomFactor.h>

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

namespace gtsam {

/**
 * MATLAB-only adapter for gtsam::CustomFactor.
 *
 * The core GTSAM CustomFactor stores a C++ callback. For MATLAB we cannot
 * store a raw mxArray* or function handle directly inside the factor because
 * the factor can outlive temporary MATLAB objects and can be copied by the
 * optimizer. Instead, the MATLAB layer registers the callback in a MATLAB-side
 * registry, passes only a numeric callback ID into C++, and this class calls
 * back into MATLAB through that registry when error() or linearize() requests
 * factor evaluation.
 */
class MatlabCustomFactor : public CustomFactor {
 private:
  /// Stable registry key owned by the MATLAB toolbox.
  size_t callback_id_;

#ifdef GTSAM_USE_TBB
  // MATLAB callback execution is not a good place to discover hidden
  // parallelism bugs. Serializing CustomFactor callback execution keeps the
  // MATLAB path predictable while debugging wrapper issues.
  std::shared_ptr<tbb::global_control> tbb_serial_guard_;
#endif

  static std::string registryFunctionName() {
    return "gtsam.customFactorRegistry";
  }

  static void destroyCallbackArgs(mxArray** args, size_t count) {
    for (size_t i = 0; i < count; ++i) {
      if (args[i]) {
        mxDestroyArray(args[i]);
      }
    }
  }

  Vector invokeCallback(const Values& values, JacobianVector* jacobians) const {
    // Call back into MATLAB through a single registry entry point:
    //   gtsam.customFactorRegistry('invoke', callbackId, values)
    // The registry owns the original MATLAB function handle and the MATLAB
    // CustomFactor proxy object, so C++ never needs to manufacture a MATLAB
    // object for `this`.
    mxArray* args[4] = {
        mxCreateString(registryFunctionName().c_str()),
        mxCreateString("invoke"),
        wrap<size_t>(callback_id_),
        wrap_shared_ptr(std::make_shared<Values>(values), "gtsam.Values", false),
    };

    mxArray* outputs[2] = {nullptr, nullptr};
    mexCallMATLAB(jacobians ? 2 : 1, outputs, 4, args, "feval");
    destroyCallbackArgs(args, 4);

    // The registry returns the residual as the first output in both calling
    // modes. Validate the dimension here so MATLAB callback bugs fail close to
    // the wrapper boundary rather than later in the optimizer.
    Vector error = unwrap<Vector>(outputs[0]);
    if (error.size() != static_cast<Eigen::Index>(this->dim())) {
      mxDestroyArray(outputs[0]);
      if (outputs[1]) {
        mxDestroyArray(outputs[1]);
      }
      mexErrMsgIdAndTxt(
          "gtsam:CustomFactor:invalidResidual",
          "CustomFactor callback returned a residual with the wrong dimension.");
    }

    if (jacobians) {
      // MATLAB must return one Jacobian block per key, in factor key order.
      // Using a cell array keeps the MATLAB side simple and matches existing
      // wrapper conventions for heterogeneous matrix outputs.
      if (!mxIsCell(outputs[1])) {
        mxDestroyArray(outputs[0]);
        mxDestroyArray(outputs[1]);
        mexErrMsgIdAndTxt(
            "gtsam:CustomFactor:invalidJacobians",
            "CustomFactor callback must return Jacobians as a cell array.");
      }

      const size_t expected = this->keys().size();
      if (mxGetNumberOfElements(outputs[1]) != expected) {
        mxDestroyArray(outputs[0]);
        mxDestroyArray(outputs[1]);
        mexErrMsgIdAndTxt(
            "gtsam:CustomFactor:invalidJacobianCount",
            "CustomFactor callback returned the wrong number of Jacobian blocks.");
      }

      jacobians->resize(expected);
      for (size_t i = 0; i < expected; ++i) {
        const mxArray* cell = mxGetCell(outputs[1], i);
        if (!cell) {
          mxDestroyArray(outputs[0]);
          mxDestroyArray(outputs[1]);
          mexErrMsgIdAndTxt(
              "gtsam:CustomFactor:emptyJacobian",
              "CustomFactor callback returned an empty Jacobian block.");
        }
        (*jacobians)[i] = unwrap<Matrix>(cell);
      }
    }

    mxDestroyArray(outputs[0]);
    if (outputs[1]) {
      mxDestroyArray(outputs[1]);
    }

    return error;
  }

 public:
  /**
   * Construct a MATLAB-backed CustomFactor.
   *
   * @param noiseModel Shared noise model used by the factor.
   * @param keys Factor keys in callback Jacobian order.
   * @param callbackId Registry ID previously allocated in MATLAB.
   */
  MatlabCustomFactor(const noiseModel::Base::shared_ptr& noiseModel,
                     const KeyVector& keys, size_t callbackId)
      : CustomFactor(
            noiseModel, keys,
            [callbackId](const CustomFactor& factor, const Values& values,
                         const JacobianVector* jacobians) {
              const auto& matlabFactor =
                  static_cast<const MatlabCustomFactor&>(factor);
              return matlabFactor.invokeCallback(
                  values, const_cast<JacobianVector*>(jacobians));
            }),
        callback_id_(callbackId)
#ifdef GTSAM_USE_TBB
        ,
        tbb_serial_guard_(std::make_shared<tbb::global_control>(
            tbb::global_control::max_allowed_parallelism, 1))
#endif
  {
    // Keep the mex module loaded for the rest of the MATLAB session once a
    // MATLAB CustomFactor exists. Without this, MATLAB can unload the wrapper
    // while callback-backed factors are still reachable through optimizer-owned
    // C++ objects, which caused unload-time crashes during debugging.
    mexLock();
  }

  ~MatlabCustomFactor() override = default;
};

}  // namespace gtsam
