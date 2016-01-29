#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/sam/BearingFactor.h>

using namespace boost::python;
using namespace gtsam;

using namespace std;

template <class VALUE>
void exportBearingFactor(const std::string& name) {
  class_<VALUE>(name, init<>());
}
