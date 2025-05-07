#include <GeographicLib/Intersect.hpp>
#include <GeographicLib/Geodesic.hpp>

extern "C" void intersect(double lat1, double lon1, double az1,
                          double lat2, double lon2, double az2,
                          double *o_dX, double *o_dY) {
  const GeographicLib::Geodesic
  geod(GeographicLib::Constants::WGS84_a(),
       GeographicLib::Constants::WGS84_f());
  const GeographicLib::Intersect intersector(geod);

  GeographicLib::Intersect::Point p0;
  p0 = intersector.Closest(lat1, lon1, az1, lat2, lon2, az2);
  *o_dX = p0.first;
  *o_dY = p0.second;
}
