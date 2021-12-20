#include "tools/workspace/pybind_wrapper/test/pybind_wrapper_test.h"

namespace anzu {

namespace sub {
double Point2::sum() const { return x_ + y_; }
}  // namespace sub

double Point3::sum() const { return x_ + y_ + z_; }

double global_func_on_base(const std::shared_ptr<PointBase>& point) {
  return point->sum();
}

}  // namespace anzu

double global_func_overloads(const std::shared_ptr<anzu::sub::Point2>& point2) {
  return point2->sum();
}

double global_func_overloads(const std::shared_ptr<anzu::Point3>& point3) {
  return point3->sum();
}
