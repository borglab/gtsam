#include <folder/path/to/MatrixViewFixture.h>

namespace gtsam {

class MatrixViewFixture {
  MatrixViewFixture();
  void acceptView(gtsam::ConstMatrixView points) const;
  gtsam::Matrix scaleView(gtsam::ConstMatrixView points, double scale = 1.0) const;
};

}
