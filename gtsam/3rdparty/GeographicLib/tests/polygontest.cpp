/**
 * \file polygontest.cpp
 * \brief Test treatment of +/-0 and +/-180
 *
 * Copyright (c) Charles Karney (2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <iostream>
#include <limits>
#include <string>
#include <GeographicLib/Math.hpp>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/PolygonArea.hpp>

using namespace std;
using namespace GeographicLib;

typedef Math::real T;

static int checkEquals(T x, T y, T d) {
  if (fabs(x - y) <= d)
    return 0;
  cout << "checkEquals fails: " << x << " != " << y << " +/- " << d << "\n";
  return 1;
}

static int checkNaN(T x) {
  using std::isnan;             // Needed for Centos 7, ubuntu 14
  if (isnan(x))
    return 0;
  cout << "checkNaN fails\n";
  return 1;
}

static int Planimeter15() {
  // Coverage tests, includes Planimeter15 - Planimeter18 (combinations of
  // reverse and sign) + calls to testpoint, testedge, geod_polygonarea.
  const Geodesic& g = Geodesic::WGS84();
  PolygonArea polygon(g);
  T lat[] = {2, 1, 3}, lon[] = {1, 2, 3};
  T perim, area, s12, azi1, azi2;
  T r = 18454562325.45119,
    a0 = 510065621724088.5093;  // ellipsoid area
  int result = 0;
  polygon.AddPoint(lat[0], lon[0]);
  polygon.AddPoint(lat[1], lon[1]);
  polygon.TestPoint(lat[2], lon[2], false, true, perim, area);
  result += checkEquals(area, r, 0.5);
  polygon.TestPoint(lat[2], lon[2], false, false, perim, area);
  result += checkEquals(area, r, 0.5);
  polygon.TestPoint(lat[2], lon[2], true, true, perim, area);
  result += checkEquals(area, -r, 0.5);
  polygon.TestPoint(lat[2], lon[2], true, false, perim, area);
  result += checkEquals(area, a0-r, 0.5);
  g.Inverse(lat[1], lon[1], lat[2], lon[2], s12, azi1, azi2);
  polygon.TestEdge(azi1, s12, false, true, perim, area);
  result += checkEquals(area, r, 0.5);
  polygon.TestEdge(azi1, s12, false, false, perim, area);
  result += checkEquals(area, r, 0.5);
  polygon.TestEdge(azi1, s12, true, true, perim, area);
  result += checkEquals(area, -r, 0.5);
  polygon.TestEdge(azi1, s12, true, false, perim, area);
  result += checkEquals(area, a0-r, 0.5);
  polygon.AddPoint(lat[2], lon[2]);
  polygon.Compute(false, true, perim, area);
  result += checkEquals(area, r, 0.5);
  polygon.Compute(false, false, perim, area);
  result += checkEquals(area, r, 0.5);
  polygon.Compute(true, true, perim, area);
  result += checkEquals(area, -r, 0.5);
  polygon.Compute(true, false, perim, area);
  result += checkEquals(area, a0-r, 0.5);
  return result;
}

static int Planimeter19() {
  // Coverage tests, includes Planimeter19 - Planimeter20 (degenerate
  // polygons) + extra cases.
  const Geodesic& g = Geodesic::WGS84();
  PolygonArea polygon(g, false);
  PolygonArea polyline(g, true);
  T perim, area;
  int result = 0;
  polygon.Compute(false, true, perim, area);
  result += checkEquals(area, 0, 0);
  result += checkEquals(perim, 0, 0);
  polygon.TestPoint(1, 1, false, true, perim, area);
  result += checkEquals(area, 0, 0);
  result += checkEquals(perim, 0, 0);
  polygon.TestEdge(90, 1000, false, true, perim, area);
  result += checkNaN(area);
  result += checkNaN(perim);
  polygon.AddPoint(1, 1);
  polygon.Compute(false, true, perim, area);
  result += checkEquals(area, 0, 0);
  result += checkEquals(perim, 0, 0);
  polyline.Compute(false, true, perim, area);
  result += checkEquals(perim, 0, 0);
  polyline.TestPoint(1, 1, false, true, perim, area);
  result += checkEquals(perim, 0, 0);
  polyline.TestEdge(90, 1000, false, true, perim, area);
  result += checkNaN(perim);
  polyline.AddPoint(1, 1);
  polyline.Compute(false, true, perim, area);
  result += checkEquals(perim, 0, 0);
  polyline.AddPoint(1, 1);
  polyline.TestEdge(90, 1000, false, true, perim, area);
  result += checkEquals(perim, 1000, 1e-10);
  polyline.TestPoint(2, 2, false, true, perim, area);
  result += checkEquals(perim, 156876.149, 0.5e-3);
  return result;
}

static int Planimeter21() {
  // Some test to add code coverage: multiple circlings of pole (includes
  // Planimeter21 - Planimeter28) + invocations via testpoint and testedge.
  const Geodesic& g = Geodesic::WGS84();
  PolygonArea polygon(g);
  T perim, area, lat = 45,
    a = 39.2144607176828184218, s = 8420705.40957178156285,
    r = 39433884866571.4277,    // Area for one circuit
    a0 = 510065621724088.5093;  // Ellipsoid area
  int result = 0, i;
  polygon.AddPoint(lat,  60);
  polygon.AddPoint(lat, 180);
  polygon.AddPoint(lat, -60);
  polygon.AddPoint(lat,  60);
  polygon.AddPoint(lat, 180);
  polygon.AddPoint(lat, -60);
  for (i = 3; i <= 4; ++i) {
    polygon.AddPoint(lat,  60);
    polygon.AddPoint(lat, 180);
    polygon.TestPoint(lat, -60, false, true, perim, area);
    result += checkEquals(area,  i*r, 0.5);
    polygon.TestPoint(lat, -60, false, false, perim, area);
    result += checkEquals(area,  i*r, 0.5);
    polygon.TestPoint(lat, -60, true, true, perim, area);
    result += checkEquals(area, -i*r, 0.5);
    polygon.TestPoint(lat, -60, true, false, perim, area);
    result += checkEquals(area, -i*r + a0, 0.5);
    polygon.TestEdge(a, s, false, true, perim, area);
    result += checkEquals(area,  i*r, 0.5);
    polygon.TestEdge(a, s, false, false, perim, area);
    result += checkEquals(area,  i*r, 0.5);
    polygon.TestEdge(a, s, true, true, perim, area);
    result += checkEquals(area, -i*r, 0.5);
    polygon.TestEdge(a, s, true, false, perim, area);
    result += checkEquals(area, -i*r + a0, 0.5);
    polygon.AddPoint(lat, -60);
    polygon.Compute(false, true, perim, area);
    result += checkEquals(area,  i*r, 0.5);
    polygon.Compute(false, false, perim, area);
    result += checkEquals(area,  i*r, 0.5);
    polygon.Compute(true, true, perim, area);
    result += checkEquals(area, -i*r, 0.5);
    polygon.Compute(true, false, perim, area);
    result += checkEquals(area, -i*r + a0, 0.5);
  }
  return result;
}

static int Planimeter29() {
  // Check fix to transitdirect vs transit zero handling inconsistency
  const Geodesic& g = Geodesic::WGS84();
  PolygonArea polygon(g);
  T perim, area;
  int result = 0;
  polygon.AddPoint(0, 0);
  polygon.AddEdge( 90, 1000);
  polygon.AddEdge(  0, 1000);
  polygon.AddEdge(-90, 1000);
  polygon.Compute(false, true, perim, area);
  // The area should be 1e6.  Prior to the fix it was 1e6 - A/2, where
  // A = ellipsoid area.
  result += checkEquals(area, 1000000.0, 0.01);
  return result;
}

int main() {
  int n = 0, i;

  i = Planimeter15(); n += i;
  if (i)
    cout << "Planimeter15 failure\n";

  i = Planimeter19(); n += i;
  if (i)
    cout << "Planimeter19 failure\n";

  i = Planimeter21(); n += i;
  if (i)
    cout << "Planimeter21 failure\n";

  i = Planimeter29(); n += i;
  if (i)
    cout << "Planimeter29 failure\n";

  if (n) {
    cout << n << " failure" << (n > 1 ? "s" : "") << "\n";
    return 1;
  }
}
