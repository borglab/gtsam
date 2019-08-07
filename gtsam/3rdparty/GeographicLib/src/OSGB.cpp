/**
 * \file OSGB.cpp
 * \brief Implementation for GeographicLib::OSGB class
 *
 * Copyright (c) Charles Karney (2010-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/OSGB.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  const char* const OSGB::letters_ = "ABCDEFGHJKLMNOPQRSTUVWXYZ";
  const char* const OSGB::digits_ = "0123456789";

  const TransverseMercator& OSGB::OSGBTM() {
    static const TransverseMercator osgbtm(MajorRadius(), Flattening(),
                                           CentralScale());
    return osgbtm;
  }

  Math::real OSGB::computenorthoffset() {
    real x, y;
    static const real northoffset =
      ( OSGBTM().Forward(real(0), OriginLatitude(), real(0), x, y),
        FalseNorthing() - y );
    return northoffset;
  }

  void OSGB::GridReference(real x, real y, int prec, std::string& gridref) {
    CheckCoords(x, y);
    if (!(prec >= 0 && prec <= maxprec_))
      throw GeographicErr("OSGB precision " + Utility::str(prec)
                          + " not in [0, "
                          + Utility::str(int(maxprec_)) + "]");
    if (Math::isnan(x) || Math::isnan(y)) {
      gridref = "INVALID";
      return;
    }
    char grid[2 + 2 * maxprec_];
    int
      xh = int(floor(x / tile_)),
      yh = int(floor(y / tile_));
    real
      xf = x - tile_ * xh,
      yf = y - tile_ * yh;
    xh += tileoffx_;
    yh += tileoffy_;
    int z = 0;
    grid[z++] = letters_[(tilegrid_ - (yh / tilegrid_) - 1)
                        * tilegrid_ + (xh / tilegrid_)];
    grid[z++] = letters_[(tilegrid_ - (yh % tilegrid_) - 1)
                        * tilegrid_ + (xh % tilegrid_)];
    // Need extra real because, since C++11, pow(float, int) returns double
    real mult = real(pow(real(base_), max(tilelevel_ - prec, 0)));
    int
      ix = int(floor(xf / mult)),
      iy = int(floor(yf / mult));
    for (int c = min(prec, int(tilelevel_)); c--;) {
      grid[z + c] = digits_[ ix % base_ ];
      ix /= base_;
      grid[z + c + prec] = digits_[ iy % base_ ];
      iy /= base_;
    }
    if (prec > tilelevel_) {
      xf -= floor(xf / mult);
      yf -= floor(yf / mult);
      mult = real(pow(real(base_), prec - tilelevel_));
      ix = int(floor(xf * mult));
      iy = int(floor(yf * mult));
      for (int c = prec - tilelevel_; c--;) {
        grid[z + c + tilelevel_] = digits_[ ix % base_ ];
        ix /= base_;
        grid[z + c + tilelevel_ + prec] = digits_[ iy % base_ ];
        iy /= base_;
      }
    }
    int mlen = z + 2 * prec;
    gridref.resize(mlen);
    copy(grid, grid + mlen, gridref.begin());
  }

  void OSGB::GridReference(const std::string& gridref,
                           real& x, real& y, int& prec,
                           bool centerp) {
    int
      len = int(gridref.size()),
      p = 0;
    if (len >= 2 &&
        toupper(gridref[0]) == 'I' &&
        toupper(gridref[1]) == 'N') {
      x = y = Math::NaN();
      prec = -2;                // For compatibility with MGRS::Reverse.
      return;
    }
    char grid[2 + 2 * maxprec_];
    for (int i = 0; i < len; ++i) {
      if (!isspace(gridref[i])) {
        if (p >= 2 + 2 * maxprec_)
          throw GeographicErr("OSGB string " + gridref + " too long");
        grid[p++] = gridref[i];
      }
    }
    len = p;
    p = 0;
    if (len < 2)
      throw GeographicErr("OSGB string " + gridref + " too short");
    if (len % 2)
      throw GeographicErr("OSGB string " + gridref +
                          " has odd number of characters");
    int
      xh = 0,
      yh = 0;
    while (p < 2) {
      int i = Utility::lookup(letters_, grid[p++]);
      if (i < 0)
        throw GeographicErr("Illegal prefix character " + gridref);
      yh = yh * tilegrid_ + tilegrid_ - (i / tilegrid_) - 1;
      xh = xh * tilegrid_ + (i % tilegrid_);
    }
    xh -= tileoffx_;
    yh -= tileoffy_;

    int prec1 = (len - p)/2;
    real
      unit = tile_,
      x1 = unit * xh,
      y1 = unit * yh;
    for (int i = 0; i < prec1; ++i) {
      unit /= base_;
      int
        ix = Utility::lookup(digits_, grid[p + i]),
        iy = Utility::lookup(digits_, grid[p + i + prec1]);
      if (ix < 0 || iy < 0)
        throw GeographicErr("Encountered a non-digit in " + gridref);
      x1 += unit * ix;
      y1 += unit * iy;
    }
    if (centerp) {
      x1 += unit/2;
      y1 += unit/2;
    }
    x = x1;
    y = y1;
    prec = prec1;
  }

  void OSGB::CheckCoords(real x, real y) {
    // Limits are all multiples of 100km and are all closed on the lower end
    // and open on the upper end -- and this is reflected in the error
    // messages.  NaNs are let through.
    if (x < minx_ || x >= maxx_)
      throw GeographicErr("Easting " + Utility::str(int(floor(x/1000)))
                          + "km not in OSGB range ["
                          + Utility::str(minx_/1000) + "km, "
                          + Utility::str(maxx_/1000) + "km)");
    if (y < miny_ || y >= maxy_)
      throw GeographicErr("Northing " + Utility::str(int(floor(y/1000)))
                          + "km not in OSGB range ["
                          + Utility::str(miny_/1000) + "km, "
                          + Utility::str(maxy_/1000) + "km)");
  }

} // namespace GeographicLib
