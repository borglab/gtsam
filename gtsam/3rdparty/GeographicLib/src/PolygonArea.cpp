/**
 * \file PolygonArea.cpp
 * \brief Implementation for GeographicLib::PolygonAreaT class
 *
 * Copyright (c) Charles Karney (2010-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/PolygonArea.hpp>

namespace GeographicLib {

  using namespace std;

  template <class GeodType>
  void PolygonAreaT<GeodType>::AddPoint(real lat, real lon) {
    lat = Math::LatFix(lat);
    lon = Math::AngNormalize(lon);
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

  template <class GeodType>
  void PolygonAreaT<GeodType>::AddEdge(real azi, real s) {
    if (_num) {                 // Do nothing if _num is zero
      real lat, lon, S12, t;
      _earth.GenDirect(_lat1, _lon1, azi, false, s, _mask,
                       lat, lon, t, t, t, t, t, S12);
      _perimetersum += s;
      if (!_polyline) {
        _areasum += S12;
        _crossings += transitdirect(_lon1, lon);
        lon = Math::AngNormalize(lon);
      }
      _lat1 = lat; _lon1 = lon;
      ++_num;
    }
  }

  template <class GeodType>
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
    if (crossings & 1)
      tempsum += (tempsum < 0 ? 1 : -1) * _area0/2;
    // area is with the clockwise sense.  If !reverse convert to
    // counter-clockwise convention.
    if (!reverse)
      tempsum *= -1;
    // If sign put area in (-area0/2, area0/2], else put area in [0, area0)
    if (sign) {
      if (tempsum > _area0/2)
        tempsum -= _area0;
      else if (tempsum <= -_area0/2)
        tempsum += _area0;
    } else {
      if (tempsum >= _area0)
        tempsum -= _area0;
      else if (tempsum < 0)
        tempsum += _area0;
    }
    area = 0 + tempsum();
    return _num;
  }

  template <class GeodType>
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

    if (crossings & 1)
      tempsum += (tempsum < 0 ? 1 : -1) * _area0/2;
    // area is with the clockwise sense.  If !reverse convert to
    // counter-clockwise convention.
    if (!reverse)
      tempsum *= -1;
    // If sign put area in (-area0/2, area0/2], else put area in [0, area0)
    if (sign) {
      if (tempsum > _area0/2)
        tempsum -= _area0;
      else if (tempsum <= -_area0/2)
        tempsum += _area0;
    } else {
      if (tempsum >= _area0)
        tempsum -= _area0;
      else if (tempsum < 0)
        tempsum += _area0;
    }
    area = 0 + tempsum;
    return num;
  }

  template <class GeodType>
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
      lon = Math::AngNormalize(lon);
      _earth.GenInverse(lat, lon, _lat0, _lon0, _mask,
                        s12, t, t, t, t, t, S12);
      perimeter += s12;
      tempsum += S12;
      crossings += transit(lon, _lon0);
    }

    if (crossings & 1)
      tempsum += (tempsum < 0 ? 1 : -1) * _area0/2;
    // area is with the clockwise sense.  If !reverse convert to
    // counter-clockwise convention.
    if (!reverse)
      tempsum *= -1;
    // If sign put area in (-area0/2, area0/2], else put area in [0, area0)
    if (sign) {
      if (tempsum > _area0/2)
        tempsum -= _area0;
      else if (tempsum <= -_area0/2)
        tempsum += _area0;
    } else {
      if (tempsum >= _area0)
        tempsum -= _area0;
      else if (tempsum < 0)
        tempsum += _area0;
    }
    area = 0 + tempsum;
    return num;
  }

  template class GEOGRAPHICLIB_EXPORT PolygonAreaT<Geodesic>;
  template class GEOGRAPHICLIB_EXPORT PolygonAreaT<GeodesicExact>;
  template class GEOGRAPHICLIB_EXPORT PolygonAreaT<Rhumb>;

} // namespace GeographicLib
