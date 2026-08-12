#include <gtsam/base/Matrix.h>

namespace gtwrap {
namespace internal {

template <>
struct PyArgPolicy<gtsam::ConstMatrixView> {
  static pybind11::arg make(const char* name) {
    pybind11::arg arg(name);
    arg.noconvert();
    return arg;
  }
};

}  // namespace internal
}  // namespace gtwrap
