#include <folder/path/to/ReturnPolicyFixture.h>

class ReturnPolicyFixture {
  gtsam::Matrix return_value(const gtsam::Matrix& value) const;
  const gtsam::Matrix& return_const_ref(const gtsam::Matrix& value) const;
  gtsam::Matrix& return_mutable_ref(const gtsam::Matrix& value);
};
