/* Please refer to:
 * https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
 * These are required to save one copy operation on Python calls.
 *
 * NOTES
 * =================
 *
 * `PYBIND11_MAKE_OPAQUE` will mark the type as "opaque" for the pybind11
 * automatic STL binding, such that the raw objects can be accessed in Python.
 * Without this they will be automatically converted to a Python object, and all
 * mutations on Python side will not be reflected on C++.
 */

// TODO(fan): This is to fix the Argument-dependent lookup (ADL) of std::pair.
// We should find a way to NOT do this.
namespace std {
using gtsam::operator<<;
}

#include "python/gtsam/preamble/arg_policies.h"

#include <gtsam/navigation/ImuFactor.h>

namespace gtsam {

/** Python-visible ManifoldPreintegration PIM with an unambiguous predict. */
class PreintegratedImuMeasurementsManifold
    : public PreintegratedImuMeasurementsT<ManifoldPreintegration> {
 public:
  using Base = PreintegratedImuMeasurementsT<ManifoldPreintegration>;
  using Base::Base;

  NavState predict(const NavState& state_i,
                   const imuBias::ConstantBias& bias_i,
                   OptionalJacobian<9, 9> H1 = {},
                   OptionalJacobian<9, 6> H2 = {}) const {
    return Base::predict(state_i, bias_i, H1, H2);
  }
};

/** Python-visible TangentPreintegration PIM with an unambiguous predict. */
class PreintegratedImuMeasurementsTangent
    : public PreintegratedImuMeasurementsT<TangentPreintegration> {
 public:
  using Base = PreintegratedImuMeasurementsT<TangentPreintegration>;
  using Base::Base;

  NavState predict(const NavState& state_i,
                   const imuBias::ConstantBias& bias_i,
                   OptionalJacobian<9, 9> H1 = {},
                   OptionalJacobian<9, 6> H2 = {}) const {
    return Base::predict(state_i, bias_i, H1, H2);
  }
};

/** Python-visible LieGroupPreintegration PIM with an unambiguous predict. */
class PreintegratedImuMeasurementsLieGroup
    : public PreintegratedImuMeasurementsT<LieGroupPreintegration> {
 public:
  using Base = PreintegratedImuMeasurementsT<LieGroupPreintegration>;
  using Base::Base;

  NavState predict(const NavState& state_i,
                   const imuBias::ConstantBias& bias_i,
                   OptionalJacobian<9, 9> H1 = {},
                   OptionalJacobian<9, 6> H2 = {}) const {
    return Base::predict(state_i, bias_i, H1, H2);
  }
};

}  // namespace gtsam
