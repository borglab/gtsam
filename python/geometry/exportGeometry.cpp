#include <boost/python.hpp>
#include <boost/cstdint.hpp>

void exportPoint2();
void exportRot2();
void exportPose2();

BOOST_PYTHON_MODULE(libgeometry){
  using namespace boost::python;
  exportPoint2();
  exportRot2();
  exportPose2();
}