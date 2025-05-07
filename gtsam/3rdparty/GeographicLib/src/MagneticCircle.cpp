/**
 * \file MagneticCircle.cpp
 * \brief Implementation for GeographicLib::MagneticCircle class
 *
 * Copyright (c) Charles Karney (2011-2021) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/MagneticCircle.hpp>
#include <fstream>
#include <sstream>
#include <GeographicLib/Geocentric.hpp>

namespace GeographicLib {

  using namespace std;

  void MagneticCircle::FieldGeocentric(real slam, real clam,
                                       real& BX, real& BY, real& BZ,
                                       real& BXt, real& BYt, real& BZt) const {
    real BXc = 0, BYc = 0, BZc = 0;
    _circ0(slam, clam, BX, BY, BZ);
    _circ1(slam, clam, BXt, BYt, BZt);
    if (_constterm)
      _circ2(slam, clam, BXc, BYc, BZc);
    if (_interpolate) {
      BXt = (BXt - BX) / _dt0;
      BYt = (BYt - BY) / _dt0;
      BZt = (BZt - BZ) / _dt0;
    }
    BX += _t1 * BXt + BXc;
    BY += _t1 * BYt + BYc;
    BZ += _t1 * BZt + BZc;

    BXt *= - _a;
    BYt *= - _a;
    BZt *= - _a;

    BX *= - _a;
    BY *= - _a;
    BZ *= - _a;
  }

  void MagneticCircle::FieldGeocentric(real lon,
                                       real& BX, real& BY, real& BZ,
                                       real& BXt, real& BYt, real& BZt) const {
    real slam, clam;
    Math::sincosd(lon, slam, clam);
    FieldGeocentric(slam, clam, BX, BY, BZ, BXt, BYt, BZt);
  }

  void MagneticCircle::Field(real lon, bool diffp,
                             real& Bx, real& By, real& Bz,
                             real& Bxt, real& Byt, real& Bzt) const {
    real slam, clam;
    Math::sincosd(lon, slam, clam);
    real M[Geocentric::dim2_];
    Geocentric::Rotation(_sphi, _cphi, slam, clam, M);
    real BX, BY, BZ, BXt, BYt, BZt; // Components in geocentric basis
    FieldGeocentric(slam, clam, BX, BY, BZ, BXt, BYt, BZt);
    if (diffp)
      Geocentric::Unrotate(M, BXt, BYt, BZt, Bxt, Byt, Bzt);
    Geocentric::Unrotate(M, BX, BY, BZ, Bx, By, Bz);
  }

} // namespace GeographicLib
