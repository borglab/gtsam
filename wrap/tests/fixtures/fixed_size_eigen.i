#include <folder/path/to/FixedSizeEigenFixture.h>

namespace gtsam {

class FixedSizeEigenFixture {
  FixedSizeEigenFixture(const gtsam::Matrix3& matrix,
                        const gtsam::Vector10& vector);

  gtsam::Matrix3 matrix(const gtsam::Matrix3& value) const;
  gtsam::Matrix36 rectangular(const gtsam::Matrix36& value) const;
  gtsam::Vector10 vector(const gtsam::Vector10& value) const;
  pair<gtsam::Matrix3, gtsam::Vector10> pairValues() const;
  std::optional<gtsam::Matrix3> optionalMatrix() const;

  gtsam::Matrix3 matrixProperty;
  gtsam::Vector10 vectorProperty;
};

}
