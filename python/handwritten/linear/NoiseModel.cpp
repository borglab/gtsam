#include <boost/python.hpp>
#include <gtsam/linear/NoiseModel.h>

using namespace boost::python;
using namespace gtsam;

void exportNoiseModels(){
  // Diagonal Noise Model, no constructor
  class_<noiseModel::Diagonal>("DiagonalNoiseModel", no_init)
  .def("Sigmas", &noiseModel::Diagonal::Sigmas)
  .staticmethod("Sigmas")
  ;
}