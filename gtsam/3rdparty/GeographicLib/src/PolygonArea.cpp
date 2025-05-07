/**
 * \file PolygonArea.cpp
 * \brief Implementation for GeographicLib::PolygonAreaT class
 *
 * Copyright (c) Charles Karney (2010-2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/PolygonArea.hpp>

namespace GeographicLib {

  using namespace std;

  template<class GeodType>
  int PolygonAreaT<GeodType>::transit(real lon1, real lon2) {
    // Return 1 or -1 if crossing prime meridian in east or west direction.
    // Otherwise return zero.  longitude = +/-0 considered to be positive.
    // This is (should be?) compatible with transitdirect which computes
    // exactly the parity of
    //   int(floor((lon1 + lon12) / 360)) - int(floor(lon1 / 360)))
    real lon12 = Math::AngDiff(lon1, lon2);
    lon1 = Math::AngNormalize(lon1);
    lon2 = Math::AngNormalize(lon2);
    // N.B. lon12 == 0 gives cross = 0
    return
      // edge case lon1 = 180, lon2 = 360->0, lon12 = 180 to give 1
      lon12 > 0 && ((lon1 < 0 && lon2 >= 0) ||
                    // lon12 > 0 && lon1 > 0 && lon2 == 0 implies lon1 == 180
                    (lon1 > 0 && lon2 == 0)) ? 1 :
      // non edge case lon1 = -180, lon2 = -360->-0, lon12 = -180
      (lon12 < 0 && lon1 >= 0 && lon2 < 0 ? -1 : 0);
    // This was the old method (treating +/- 0 as negative).  However, with the
    // new scheme for handling longitude differences this fails on:
    // lon1 = -180, lon2 = -360->-0, lon12 = -180 gives 0 not -1.
    //    return
    //      lon1 <= 0 && lon2 > 0 && lon12 > 0 ? 1 :
    //      (lon2 <= 0 && lon1 > 0 && lon12 < 0 ? -1 : 0);
  }

  // an alternate version of transit to deal with longitudes in the direct
  // problem.
  template<class GeodType>
  int PolygonAreaT<GeodType>::transitdirect(real lon1, real lon2) {
    // Compute exactly the parity of
    //   int(floor(lon2 / 360)) - int(floor(lon1 / 360))
    // C++ C remainder -> [-360, 360]
    // Java % -> (-720, 720) switch to IEEEremainder -> [-360, 360]
    // JS % -> (-720, 720)
    // Python fmod -> (-720, 720) swith to Math.remainder
    // Fortran, Octave skip
    // If mod function gives result in [-360, 360]
    // [0, 360) -> 0; [-360, 0) or 360 -> 1
    // If mod function gives result in (-720, 720)
    // [0, 360) or [-inf, -360) -> 0; [-360, 0) or [360, inf) -> 1
    lon1 = remainder(lon1, real(2 * Math::td));
    lon2 = remainder(lon2, real(2 * Math::td));
    return ( (lon2 >= 0 && lon2 < Math::td ? 0 : 1) -
             (lon1 >= 0 && lon1 < Math::td ? 0 : 1) );
  }

  template<class GeodType>
  void PolygonAreaT<GeodType>::AddPoint(real lat, real lon) {
    if (_num == 0) {
      _lat0 = _lat1 = lat;
      _lon0 = _lon1 = lon;
    } else {
      real s12, S12, t;
      _earth.GenInverse(_lat1, _lon1, lat, lon, _mask,
                        s12, t, t, t, t, t, S12);
      _perimetersum += s12;
      if (!_polyline) {
        _areasum += S12;
        _crossings += transit(_lon1, lon);
      }
      _lat1 = lat; _lon1 = lon;
    }
    ++_num;
  }

  template<class GeodType>
  void PolygonAreaT<GeodType>::AddEdge(real azi, real s) {
    if (_num) {                 // Do nothing if _num is zero
      real lat, lon, S12, t;
      _earth.GenDirect(_lat1, _lon1, azi, false, s, _mask,
                       lat, lon, t, t, t, t, t, S12);
      _perimetersum += s;
      if (!_polyline) {
        _areasum += S12;
        _crossings += transitdirect(_lon1, lon);
      }
      _lat1 = lat; _lon1 = lon;
      ++_num;
    }
  }

  template<class GeodType>
  unsigned PolygonAreaT<GeodType>::Compute(bool reverse, bool sign,
                                           real& perimeter, real& area) const
  {
    real s12, S12, t;
    if (_num < 2) {
      perimeter = 0;
      if (!_polyline)
        area = 0;
      return _num;
    }
    if (_polyline) {
      perimeter = _perimetersum();
      return _num;
    }
    _earth.GenInverse(_lat1, _lon1, _lat0, _lon0, _mask,
                      s12, t, t, t, t, t, S12);
    perimeter = _perimetersum(s12);
    Accumulator<> tempsum(_areasum);
    tempsum += S12;
    int crossings = _crossings + transit(_lon1, _lon0);
    AreaReduce(tempsum, crossings, reverse, sign);
    area = real(0) + tempsum();
    return _num;
  }

  template<class GeodType>
  unsigned PolygonAreaT<GeodType>::TestPoint(real lat, real lon,
                                             bool reverse, bool sign,
                                             real& perimeter, real& area) const
  {
    if (_num == 0) {
      perimeter = 0;
      if (!_polyline)
        area = 0;
      return 1;
    }
    perimeter = _perimetersum();
    real tempsum = _polyline ? 0 : _areasum();
    int crossings = _crossings;
    unsigned num = _num + 1;
    for (int i = 0; i < (_polyline ? 1 : 2); ++i) {
      real s12, S12, t;
      _earth.GenInverse(i == 0 ? _lat1 : lat, i == 0 ? _lon1 : lon,
                        i != 0 ? _lat0 : lat, i != 0 ? _lon0 : lon,
                        _mask, s12, t, t, t, t, t, S12);
      perimeter += s12;
      if (!_polyline) {
        tempsum += S12;
        crossings += transit(i == 0 ? _lon1 : lon,
                             i != 0 ? _lon0 : lon);
      }
    }

    if (_polyline)
      return num;

    AreaReduce(tempsum, crossings, reverse, sign);
    area = real(0) + tempsum;
    return num;
  }

  template<class GeodType>
  unsigned PolygonAreaT<GeodType>::TestEdge(real azi, real s,
                                            bool reverse, bool sign,
                                            real& perimeter, real& area) const
  {
    if (_num == 0) {            // we don't have a starting point!
      perimeter = Math::NaN();
      if (!_polyline)
        area = Math::NaN();
      return 0;
    }
    unsigned num = _num + 1;
    perimeter = _perimetersum() + s;
    if (_polyline)
      return num;

    real tempsum =  _areasum();
    int crossings = _crossings;
    {
      real lat, lon, s12, S12, t;
      _earth.GenDirect(_lat1, _lon1, azi, false, s, _mask,
                       lat, lon, t, t, t, t, t, S12);
      tempsum += S12;
      crossings += transitdirect(_lon1, lon);
      _earth.GenInverse(lat, lon, _lat0, _lon0, _mask,
                        s12, t, t, t, t, t, S12);
      perimeter += s12;
      tempsum += S12;
      crossings += transit(lon, _lon0);
    }

    AreaReduce(tempsum, crossings, reverse, sign);
    area = real(0) + tempsum;
    return num;
  }

  template<class GeodType>
  template<typename T>
  void PolygonAreaT<GeodType>::AreaReduce(T& area, int crossings,
                                          bool reverse, bool sign) const {
    Remainder(area);
    if (crossings & 1) area += (area < 0 ? 1 : -1) * _area0/2;
    // area is with the clockwise sense.  If !reverse convert to
    // counter-clockwise convention.
    if (!reverse) area *= -1;
    // If sign put area in (-_area0/2, _area0/2], else put area in [0, _area0)
    if (sign) {
      if (area > _area0/2)
        area -= _area0;
      else if (area <= -_area0/2)
        area += _area0;
    } else {
      if (area >= _area0)
        area -= _area0;
      else if (area < 0)
        area += _area0;
    }
  }

  template class GEOGRAPHICLIB_EXPORT PolygonAreaT<Geodesic>;
  template class GEOGRAPHICLIB_EXPORT PolygonAreaT<GeodesicExact>;
  template class GEOGRAPHICLIB_EXPORT PolygonAreaT<Rhumb>;

} // namespace GeographicLib
