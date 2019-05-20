#include <boost/python.hpp>
#include <GeographicLib/Geoid.hpp>

using namespace boost::python;
using namespace GeographicLib;

double EllipsoidHeight(Geoid& geoid,
                       double lat, double lon, double hmsl) {
  return hmsl + Geoid::GEOIDTOELLIPSOID * geoid(lat, lon);
}

BOOST_PYTHON_MODULE(PyGeographicLib) {

  class_<Geoid, boost::noncopyable>("Geoid", init<std::string>())
    .def("EllipsoidHeight", &EllipsoidHeight,
         "Return geoid height:\n\
    input: lat, lon, height_above_geoid\n\
    output: height_above_ellipsoid")
    ;

}
