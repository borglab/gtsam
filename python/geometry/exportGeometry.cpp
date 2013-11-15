#include <boost/python.hpp>
#include <boost/cstdint.hpp>

void exportPoint2();

BOOST_PYTHON_MODULE(libgeometry){
  using namespace boost::python;

  exportPoint2();
}