//*************************************************************************
// Custom Factor wrapping
//*************************************************************************

namespace gtsam {

#include <gtsam/nonlinear/CustomFactor.h>
virtual class CustomFactor : gtsam::NoiseModelFactor {
  /*
   * Note CustomFactor will not be wrapped for MATLAB, as there is no supporting
   * machinery there. This is achieved by adding `gtsam::CustomFactor` to the
   * ignore list in `matlab/CMakeLists.txt`.
   */
  CustomFactor();
  /*
   * Example:
   * ```
   * def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
   *    <calculated error>
   *    if not H is None:
   *        <calculate the Jacobian>
   *        H[0] = J1 # 2-d numpy array for a Jacobian block
   *        H[1] = J2
   *        ...
   *    return error # 1-d numpy array
   *
   * cf = CustomFactor(noise_model, keys, error_func)
   * ```
   */
  CustomFactor(const gtsam::SharedNoiseModel& noiseModel,
               const gtsam::KeyVector& keys,
               const gtsam::CustomErrorFunction& errorFunction);

  void print(string s = "",
             gtsam::KeyFormatter keyFormatter = gtsam::DefaultKeyFormatter);
};

}