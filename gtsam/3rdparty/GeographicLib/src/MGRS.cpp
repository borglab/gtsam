/**
 * \file MGRS.cpp
 * \brief Implementation for GeographicLib::MGRS class
 *
 * Copyright (c) Charles Karney (2008-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  const Math::real MGRS::eps_ =
    // 25 = ceil(log_2(2e7)) -- use half circumference here because northing
    // 195e5 is a legal in the "southern" hemisphere.
    pow(real(0.5), numeric_limits<real>::digits - 25);
  const Math::real MGRS::angeps_ =
    // 7 = ceil(log_2(90))
    pow(real(0.5), numeric_limits<real>::digits - 7);
  const string MGRS::hemispheres_ = "SN";
  const string MGRS::utmcols_[3] = { "ABCDEFGH", "JKLMNPQR", "STUVWXYZ" };
  const string MGRS::utmrow_ = "ABCDEFGHJKLMNPQRSTUV";
  const string MGRS::upscols_[4] =
    { "JKLPQRSTUXYZ", "ABCFGHJKLPQR", "RSTUXYZ", "ABCFGHJ" };
  const string MGRS::upsrows_[2] =
    { "ABCDEFGHJKLMNPQRSTUVWXYZ", "ABCDEFGHJKLMNP" };
  const string MGRS::latband_ = "CDEFGHJKLMNPQRSTUVWX";
  const string MGRS::upsband_ = "ABYZ";
  const string MGRS::digits_ = "0123456789";

  const int MGRS::mineasting_[4] =
    { minupsSind_, minupsNind_, minutmcol_, minutmcol_ };
  const int MGRS::maxeasting_[4] =
    { maxupsSind_, maxupsNind_, maxutmcol_, maxutmcol_ };
  const int MGRS::minnorthing_[4] =
    { minupsSind_, minupsNind_,
      minutmSrow_, minutmSrow_ - (maxutmSrow_ - minutmNrow_) };
  const int MGRS::maxnorthing_[4] =
    { maxupsSind_, maxupsNind_,
      maxutmNrow_ + (maxutmSrow_ - minutmNrow_), maxutmNrow_ };

  void MGRS::Forward(int zone, bool northp, real x, real y, real lat,
                     int prec, std::string& mgrs) {
    if (zone == UTMUPS::INVALID ||
        Math::isnan(x) || Math::isnan(y) || Math::isnan(lat)) {
      prec = -1;
      mgrs = "INVALID";
      return;
    }
    bool utmp = zone != 0;
    CheckCoords(utmp, northp, x, y);
    if (!(zone >= UTMUPS::MINZONE && zone <= UTMUPS::MAXZONE))
      throw GeographicErr("Zone " + Utility::str(zone) + " not in [0,60]");
    if (!(prec >= 0 && prec <= maxprec_))
      throw GeographicErr("MGRS precision " + Utility::str(prec)
                          + " not in [0, "
                          + Utility::str(int(maxprec_)) + "]");
    // Fixed char array for accumulating string.  Allow space for zone, 3 block
    // letters, easting + northing.  Don't need to allow for terminating null.
    char mgrs1[2 + 3 + 2 * maxprec_];
    int
      zone1 = zone - 1,
      z = utmp ? 2 : 0,
      mlen = z + 3 + 2 * prec;
    if (utmp) {
      mgrs1[0] = digits_[ zone / base_ ];
      mgrs1[1] = digits_[ zone % base_ ];
      // This isn't necessary...!  Keep y non-neg
      // if (!northp) y -= maxutmSrow_ * tile_;
    }
    int
      xh = int(floor(x)) / tile_,
      yh = int(floor(y)) / tile_;
    real
      xf = x - tile_ * xh,
      yf = y - tile_ * yh;
    if (utmp) {
      int
        // Correct fuzziness in latitude near equator
        iband = abs(lat) > angeps_ ? LatitudeBand(lat) : (northp ? 0 : -1),
        icol = xh - minutmcol_,
        irow = UTMRow(iband, icol, yh % utmrowperiod_);
      if (irow != yh - (northp ? minutmNrow_ : maxutmSrow_))
        throw GeographicErr("Latitude " + Utility::str(lat)
                            + " is inconsistent with UTM coordinates");
      mgrs1[z++] = latband_[10 + iband];
      mgrs1[z++] = utmcols_[zone1 % 3][icol];
      mgrs1[z++] = utmrow_[(yh + (zone1 & 1 ? utmevenrowshift_ : 0))
                         % utmrowperiod_];
    } else {
      bool eastp = xh >= upseasting_;
      int iband = (northp ? 2 : 0) + (eastp ? 1 : 0);
      mgrs1[z++] = upsband_[iband];
      mgrs1[z++] = upscols_[iband][xh - (eastp ? upseasting_ :
                                         (northp ? minupsNind_ : minupsSind_))];
      mgrs1[z++] = upsrows_[northp][yh - (northp ? minupsNind_ : minupsSind_)];
    }
    real mult = pow(real(base_), max(tilelevel_ - prec, 0));
    int
      ix = int(floor(xf / mult)),
      iy = int(floor(yf / mult));
    for (int c = min(prec, int(tilelevel_)); c--;) {
      mgrs1[z + c] = digits_[ ix % base_ ];
      ix /= base_;
      mgrs1[z + c + prec] = digits_[ iy % base_ ];
      iy /= base_;
    }
    if (prec > tilelevel_) {
      xf -= floor(xf / mult);
      yf -= floor(yf / mult);
      mult = pow(real(base_), prec - tilelevel_);
      ix = int(floor(xf * mult));
      iy = int(floor(yf * mult));
      for (int c = prec - tilelevel_; c--;) {
        mgrs1[z + c + tilelevel_] = digits_[ ix % base_ ];
        ix /= base_;
        mgrs1[z + c + tilelevel_ + prec] = digits_[ iy % base_ ];
        iy /= base_;
      }
    }
    mgrs.resize(mlen);
    copy(mgrs1, mgrs1 + mlen, mgrs.begin());
  }

  void MGRS::Forward(int zone, bool northp, real x, real y,
                     int prec, std::string& mgrs) {
    real lat, lon;
    if (zone > 0) {
      // Does a rough estimate for latitude determine the latitude band?
      real ys = northp ? y : y - utmNshift_;
      // A cheap calculation of the latitude which results in an "allowed"
      // latitude band would be
      //   lat = ApproxLatitudeBand(ys) * 8 + 4;
      //
      // Here we do a more careful job using the band letter corresponding to
      // the actual latitude.
      ys /= tile_;
      if (abs(ys) < 1)
        lat = 0.9 * ys;         // accurate enough estimate near equator
      else {
        real
          // The poleward bound a fit from above of lat(x,y)
          // for x = 500km and y = [0km, 950km]
          latp = real(0.901) * ys + (ys > 0 ? 1 : -1) * real(0.135),
          // The equatorward bound is a fit from below of lat(x,y)
          // for x = 900km and y = [0km, 950km]
          late = real(0.902) * ys * (1 - real(1.85e-6) * ys * ys);
        if (LatitudeBand(latp) == LatitudeBand(late))
          lat = latp;
        else
          // bounds straddle a band boundary so need to compute lat accurately
          UTMUPS::Reverse(zone, northp, x, y, lat, lon);
      }
    } else
      // Latitude isn't needed for UPS specs or for INVALID
      lat = 0;
    Forward(zone, northp, x, y, lat, prec, mgrs);
  }

  void MGRS::Reverse(const std::string& mgrs,
                     int& zone, bool& northp, real& x, real& y,
                     int& prec, bool centerp) {
    int
      p = 0,
      len = int(mgrs.size());
    if (len >= 3 &&
        toupper(mgrs[0]) == 'I' &&
        toupper(mgrs[1]) == 'N' &&
        toupper(mgrs[2]) == 'V') {
      zone = UTMUPS::INVALID;
      northp = false;
      x = y = Math::NaN<real>();
      prec = -1;
      return;
    }
    int zone1 = 0;
    while (p < len) {
      int i = Utility::lookup(digits_, mgrs[p]);
      if (i < 0)
        break;
      zone1 = 10 * zone1 + i;
      ++p;
    }
    if (p > 0 && !(zone1 >= UTMUPS::MINUTMZONE && zone1 <= UTMUPS::MAXUTMZONE))
      throw GeographicErr("Zone " + Utility::str(zone1) + " not in [1,60]");
    if (p > 2)
      throw GeographicErr("More than 2 digits_ at start of MGRS "
                          + mgrs.substr(0, p));
    if (len - p < 3)
      throw GeographicErr("MGRS string too short " + mgrs);
    bool utmp = zone1 != UTMUPS::UPS;
    int zonem1 = zone1 - 1;
    const string& band = utmp ? latband_ : upsband_;
    int iband = Utility::lookup(band, mgrs[p++]);
    if (iband < 0)
      throw GeographicErr("Band letter " + Utility::str(mgrs[p-1]) + " not in "
                          + (utmp ? "UTM" : "UPS") + " set " + band);
    bool northp1 = iband >= (utmp ? 10 : 2);
    const string& col = utmp ? utmcols_[zonem1 % 3] : upscols_[iband];
    const string& row = utmp ? utmrow_ : upsrows_[northp1];
    int icol = Utility::lookup(col, mgrs[p++]);
    if (icol < 0)
      throw GeographicErr("Column letter " + Utility::str(mgrs[p-1])
                          + " not in "
                          + (utmp ? "zone " + mgrs.substr(0, p-2) :
                             "UPS band " + Utility::str(mgrs[p-2]))
                          + " set " + col );
    int irow = Utility::lookup(row, mgrs[p++]);
    if (irow < 0)
      throw GeographicErr("Row letter " + Utility::str(mgrs[p-1]) + " not in "
                          + (utmp ? "UTM" :
                             "UPS " + Utility::str(hemispheres_[northp1]))
                          + " set " + row);
    if (utmp) {
      if (zonem1 & 1)
        irow = (irow + utmrowperiod_ - utmevenrowshift_) % utmrowperiod_;
      iband -= 10;
      irow = UTMRow(iband, icol, irow);
      if (irow == maxutmSrow_)
        throw GeographicErr("Block " + mgrs.substr(p-2, 2)
                            + " not in zone/band " + mgrs.substr(0, p-2));

      irow = northp1 ? irow : irow + 100;
      icol = icol + minutmcol_;
    } else {
      bool eastp = iband & 1;
      icol += eastp ? upseasting_ : (northp1 ? minupsNind_ : minupsSind_);
      irow += northp1 ? minupsNind_ : minupsSind_;
    }
    int prec1 = (len - p)/2;
    real
      unit = tile_,
      x1 = unit * icol,
      y1 = unit * irow;
    for (int i = 0; i < prec1; ++i) {
      unit /= base_;
      int
        ix = Utility::lookup(digits_, mgrs[p + i]),
        iy = Utility::lookup(digits_, mgrs[p + i + prec1]);
      if (ix < 0 || iy < 0)
        throw GeographicErr("Encountered a non-digit in " + mgrs.substr(p));
      x1 += unit * ix;
      y1 += unit * iy;
    }
    if ((len - p) % 2) {
      if (Utility::lookup(digits_, mgrs[len - 1]) < 0)
        throw GeographicErr("Encountered a non-digit in " + mgrs.substr(p));
      else
        throw GeographicErr("Not an even number of digits_ in "
                            + mgrs.substr(p));
    }
    if (prec1 > maxprec_)
      throw GeographicErr("More than " + Utility::str(2*maxprec_)
                          + " digits_ in "
                          + mgrs.substr(p));
    if (centerp) {
      x1 += unit/2;
      y1 += unit/2;
    }
    zone = zone1;
    northp = northp1;
    x = x1;
    y = y1;
    prec = prec1;
  }

  void MGRS::CheckCoords(bool utmp, bool& northp, real& x, real& y) {
    // Limits are all multiples of 100km and are all closed on the lower end
    // and open on the upper end -- and this is reflected in the error
    // messages.  However if a coordinate lies on the excluded upper end (e.g.,
    // after rounding), it is shifted down by eps_.  This also folds UTM
    // northings to the correct N/S hemisphere.
    int
      ix = int(floor(x / tile_)),
      iy = int(floor(y / tile_)),
      ind = (utmp ? 2 : 0) + (northp ? 1 : 0);
    if (! (ix >= mineasting_[ind] && ix < maxeasting_[ind]) ) {
      if (ix == maxeasting_[ind] && x == maxeasting_[ind] * tile_)
        x -= eps_;
      else
        throw GeographicErr("Easting " + Utility::str(int(floor(x/1000)))
                            + "km not in MGRS/"
                            + (utmp ? "UTM" : "UPS") + " range for "
                            + (northp ? "N" : "S" ) + " hemisphere ["
                            + Utility::str(mineasting_[ind]*tile_/1000)
                            + "km, "
                            + Utility::str(maxeasting_[ind]*tile_/1000)
                            + "km)");
    }
    if (! (iy >= minnorthing_[ind] && iy < maxnorthing_[ind]) ) {
      if (iy == maxnorthing_[ind] && y == maxnorthing_[ind] * tile_)
        y -= eps_;
      else
        throw GeographicErr("Northing " + Utility::str(int(floor(y/1000)))
                            + "km not in MGRS/"
                            + (utmp ? "UTM" : "UPS") + " range for "
                            + (northp ? "N" : "S" ) + " hemisphere ["
                            + Utility::str(minnorthing_[ind]*tile_/1000)
                            + "km, "
                            + Utility::str(maxnorthing_[ind]*tile_/1000)
                            + "km)");
    }

    // Correct the UTM northing and hemisphere if necessary
    if (utmp) {
      if (northp && iy < minutmNrow_) {
        northp = false;
        y += utmNshift_;
      } else if (!northp && iy >= maxutmSrow_) {
        if (y == maxutmSrow_ * tile_)
          // If on equator retain S hemisphere
          y -= eps_;
        else {
          northp = true;
          y -= utmNshift_;
        }
      }
    }
  }

  int MGRS::UTMRow(int iband, int icol, int irow) throw() {
    // Input is MGRS (periodic) row index and output is true row index.  Band
    // index is in [-10, 10) (as returned by LatitudeBand).  Column index
    // origin is easting = 100km.  Returns maxutmSrow_ if irow and iband are
    // incompatible.  Row index origin is equator.

    // Estimate center row number for latitude band
    // 90 deg = 100 tiles; 1 band = 8 deg = 100*8/90 tiles
    real c = 100 * (8 * iband + 4)/real(90);
    bool northp = iband >= 0;
    //  iband minrow maxrow
    //   -10    -90    -81
    //    -9    -80    -72
    //    -8    -71    -63
    //    -7    -63    -54
    //    -6    -54    -45
    //    -5    -45    -36
    //    -4    -36    -27
    //    -3    -27    -18
    //    -2    -18     -9
    //    -1     -9     -1
    //     0      0      8
    //     1      8     17
    //     2     17     26
    //     3     26     35
    //     4     35     44
    //     5     44     53
    //     6     53     62
    //     7     62     70
    //     8     71     79
    //     9     80     94
    int
      minrow = iband > -10 ?
      int(floor(c - real(4.3) - real(0.1) * northp)) : -90,
      maxrow = iband <   9 ?
      int(floor(c + real(4.4) - real(0.1) * northp)) :  94,
      baserow = (minrow + maxrow) / 2 - utmrowperiod_ / 2;
    // Add maxutmSrow_ = 5 * utmrowperiod_ to ensure operand is positive
    irow = (irow - baserow + maxutmSrow_) % utmrowperiod_ + baserow;
    if (irow < minrow || irow > maxrow) {
      // Northing = 71*100km and 80*100km intersect band boundaries
      // The following deals with these special cases.
      int
        // Fold [-10,-1] -> [9,0]
        sband = iband >= 0 ? iband : -iband - 1,
        // Fold [-90,-1] -> [89,0]
        srow = irow >= 0 ? irow : -irow - 1,
        // Fold [4,7] -> [3,0]
        scol = icol < 4 ? icol : -icol + 7;
      if ( ! ( (srow == 70 && sband == 8 && scol >= 2) ||
               (srow == 71 && sband == 7 && scol <= 2) ||
               (srow == 79 && sband == 9 && scol >= 1) ||
               (srow == 80 && sband == 8 && scol <= 1) ) )
        irow = maxutmSrow_;
    }
    return irow;
  }

} // namespace GeographicLib
