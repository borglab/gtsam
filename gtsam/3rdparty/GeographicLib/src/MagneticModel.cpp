/**
 * \file MagneticModel.cpp
 * \brief Implementation for GeographicLib::MagneticModel class
 *
 * Copyright (c) Charles Karney (2011-2021) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/MagneticModel.hpp>
#include <fstream>
#include <GeographicLib/SphericalEngine.hpp>
#include <GeographicLib/MagneticCircle.hpp>
#include <GeographicLib/Utility.hpp>

#if !defined(GEOGRAPHICLIB_DATA)
#  if defined(_WIN32)
#    define GEOGRAPHICLIB_DATA "C:/ProgramData/GeographicLib"
#  else
#    define GEOGRAPHICLIB_DATA "/usr/local/share/GeographicLib"
#  endif
#endif

#if !defined(GEOGRAPHICLIB_MAGNETIC_DEFAULT_NAME)
#  define GEOGRAPHICLIB_MAGNETIC_DEFAULT_NAME "wmm2025"
#endif

#if defined(_MSC_VER)
// Squelch warnings about unsafe use of getenv
#  pragma warning (disable: 4996)
#endif

namespace GeographicLib {

  using namespace std;

  MagneticModel::MagneticModel(const std::string& name, const std::string& path,
                               const Geocentric& earth, int Nmax, int Mmax)
    : _name(name)
    , _dir(path)
    , _description("NONE")
    , _date("UNKNOWN")
    , _t0(Math::NaN())
    , _dt0(1)
    , _tmin(Math::NaN())
    , _tmax(Math::NaN())
    , _a(Math::NaN())
    , _hmin(Math::NaN())
    , _hmax(Math::NaN())
    , _nNmodels(1)
    , _nNconstants(0)
    , _nmx(-1)
    , _mmx(-1)
    , _norm(SphericalHarmonic::SCHMIDT)
    , _earth(earth)
  {
    if (_dir.empty())
      _dir = DefaultMagneticPath();
    bool truncate = Nmax >= 0 || Mmax >= 0;
    if (truncate) {
      if (Nmax >= 0 && Mmax < 0) Mmax = Nmax;
      if (Nmax < 0) Nmax = numeric_limits<int>::max();
      if (Mmax < 0) Mmax = numeric_limits<int>::max();
    }
    ReadMetadata(_name);
    _gG.resize(_nNmodels + 1 + _nNconstants);
    _hH.resize(_nNmodels + 1 + _nNconstants);
    {
      string coeff = _filename + ".cof";
      ifstream coeffstr(coeff.c_str(), ios::binary);
      if (!coeffstr.good())
        throw GeographicErr("Error opening " + coeff);
      char id[idlength_ + 1];
      coeffstr.read(id, idlength_);
      if (!coeffstr.good())
        throw GeographicErr("No header in " + coeff);
      id[idlength_] = '\0';
      if (_id != string(id))
        throw GeographicErr("ID mismatch: " + _id + " vs " + id);
      for (int i = 0; i < _nNmodels + 1 + _nNconstants; ++i) {
        int N, M;
        if (truncate) { N = Nmax; M = Mmax; }
        SphericalEngine::coeff::readcoeffs(coeffstr, N, M, _gG[i], _hH[i],
                                           truncate);
        if (!(M < 0 || _gG[i][0] == 0))
          throw GeographicErr("A degree 0 term is not permitted");
        _harm.push_back(SphericalHarmonic(_gG[i], _hH[i], N, N, M, _a, _norm));
        _nmx = max(_nmx, _harm.back().Coefficients().nmx());
        _mmx = max(_mmx, _harm.back().Coefficients().mmx());
      }
      int pos = int(coeffstr.tellg());
      coeffstr.seekg(0, ios::end);
      if (pos != coeffstr.tellg())
        throw GeographicErr("Extra data in " + coeff);
    }
  }

  void MagneticModel::ReadMetadata(const string& name) {
    const char* spaces = " \t\n\v\f\r";
    _filename = _dir + "/" + name + ".wmm";
    ifstream metastr(_filename.c_str());
    if (!metastr.good())
      throw GeographicErr("Cannot open " + _filename);
    string line;
    getline(metastr, line);
    if (!(line.size() >= 6 && line.substr(0,5) == "WMMF-"))
      throw GeographicErr(_filename + " does not contain WMMF-n signature");
    string::size_type n = line.find_first_of(spaces, 5);
    if (n != string::npos)
      n -= 5;
    string version(line, 5, n);
    // version 2 added treatment of NumConstants.  The MagneticModel class now
    // accepts NumConstants for version = 1 or 2.  (The version logic was
    // necessary to allow old versions of GeographicLib, which didn't
    // understand NumConstants, to complain when it encountered a version 2
    // file.)
    if (!(version == "1" || version == "2"))
      throw GeographicErr("Unknown version in " + _filename + ": " + version);
    string key, val;
    while (getline(metastr, line)) {
      if (!Utility::ParseLine(line, key, val))
        continue;
      // Process key words
      if (key == "Name")
        _name = val;
      else if (key == "Description")
        _description = val;
      else if (key == "ReleaseDate")
        _date = val;
      else if (key == "Radius")
        _a = Utility::val<real>(val);
      else if (key == "Type") {
        if (!(val == "Linear" || val == "linear"))
          throw GeographicErr("Only linear models are supported");
      } else if (key == "Epoch")
        _t0 = Utility::val<real>(val);
      else if (key == "DeltaEpoch")
        _dt0 = Utility::val<real>(val);
      else if (key == "NumModels")
        _nNmodels = Utility::val<int>(val);
      else if (key == "NumConstants")
        _nNconstants = Utility::val<int>(val);
      else if (key == "MinTime")
        _tmin = Utility::val<real>(val);
      else if (key == "MaxTime")
        _tmax = Utility::val<real>(val);
      else if (key == "MinHeight")
        _hmin = Utility::val<real>(val);
      else if (key == "MaxHeight")
        _hmax = Utility::val<real>(val);
      else if (key == "Normalization") {
        if (val == "FULL" || val == "Full" || val == "full")
          _norm = SphericalHarmonic::FULL;
        else if (val == "SCHMIDT" || val == "Schmidt" || val == "schmidt")
          _norm = SphericalHarmonic::SCHMIDT;
        else
          throw GeographicErr("Unknown normalization " + val);
      } else if (key == "ByteOrder") {
        if (val == "Big" || val == "big")
          throw GeographicErr("Only little-endian ordering is supported");
        else if (!(val == "Little" || val == "little"))
          throw GeographicErr("Unknown byte ordering " + val);
      } else if (key == "ID")
        _id = val;
      // else unrecognized keywords are skipped
    }
    // Check values
    if (!(isfinite(_a) && _a > 0))
      throw GeographicErr("Reference radius must be positive");
    if (!(_t0 > 0))
      throw GeographicErr("Epoch time not defined");
    if (_tmin >= _tmax)
      throw GeographicErr("Min time exceeds max time");
    if (_hmin >= _hmax)
      throw GeographicErr("Min height exceeds max height");
    if (int(_id.size()) != idlength_)
      throw GeographicErr("Invalid ID");
    if (_nNmodels < 1)
      throw GeographicErr("NumModels must be positive");
    if (!(_nNconstants == 0 || _nNconstants == 1))
      throw GeographicErr("NumConstants must be 0 or 1");
    if (!(_dt0 > 0)) {
      if (_nNmodels > 1)
        throw GeographicErr("DeltaEpoch must be positive");
      else
        _dt0 = 1;
    }
  }

  void MagneticModel::FieldGeocentric(real t, real X, real Y, real Z,
                                      real& BX, real& BY, real& BZ,
                                      real& BXt, real& BYt, real& BZt) const {
    t -= _t0;
    int n = max(min(int(floor(t / _dt0)), _nNmodels - 1), 0);
    bool interpolate = n + 1 < _nNmodels;
    t -= n * _dt0;
    // Components in geocentric basis
    // initial values to suppress warning
    real BXc = 0, BYc = 0, BZc = 0;
    _harm[n](X, Y, Z, BX, BY, BZ);
    _harm[n + 1](X, Y, Z, BXt, BYt, BZt);
    if (_nNconstants)
      _harm[_nNmodels + 1](X, Y, Z, BXc, BYc, BZc);
    if (interpolate) {
      // Convert to a time derivative
      BXt = (BXt - BX) / _dt0;
      BYt = (BYt - BY) / _dt0;
      BZt = (BZt - BZ) / _dt0;
    }
    BX += t * BXt + BXc;
    BY += t * BYt + BYc;
    BZ += t * BZt + BZc;

    BXt = BXt * - _a;
    BYt = BYt * - _a;
    BZt = BZt * - _a;

    BX *= - _a;
    BY *= - _a;
    BZ *= - _a;
  }

  void MagneticModel::Field(real t, real lat, real lon, real h, bool diffp,
                            real& Bx, real& By, real& Bz,
                            real& Bxt, real& Byt, real& Bzt) const {
    real X, Y, Z;
    real M[Geocentric::dim2_];
    _earth.IntForward(lat, lon, h, X, Y, Z, M);
    // Components in geocentric basis
    // initial values to suppress warning
    real BX = 0, BY = 0, BZ = 0, BXt = 0, BYt = 0, BZt = 0;
    FieldGeocentric(t, X, Y, Z, BX, BY, BZ, BXt, BYt, BZt);
    if (diffp)
      Geocentric::Unrotate(M, BXt, BYt, BZt, Bxt, Byt, Bzt);
    Geocentric::Unrotate(M, BX, BY, BZ, Bx, By, Bz);
  }

  MagneticCircle MagneticModel::Circle(real t, real lat, real h) const {
    real t1 = t - _t0;
    int n = max(min(int(floor(t1 / _dt0)), _nNmodels - 1), 0);
    bool interpolate = n + 1 < _nNmodels;
    t1 -= n * _dt0;
    real X, Y, Z, M[Geocentric::dim2_];
    _earth.IntForward(lat, 0, h, X, Y, Z, M);
    // Y = 0, cphi = M[7], sphi = M[8];

    return (_nNconstants == 0 ?
            MagneticCircle(_a, _earth._f, lat, h, t,
                           M[7], M[8], t1, _dt0, interpolate,
                           _harm[n].Circle(X, Z, true),
                           _harm[n + 1].Circle(X, Z, true)) :
            MagneticCircle(_a, _earth._f, lat, h, t,
                           M[7], M[8], t1, _dt0, interpolate,
                           _harm[n].Circle(X, Z, true),
                           _harm[n + 1].Circle(X, Z, true),
                           _harm[_nNmodels + 1].Circle(X, Z, true)));
  }

  void MagneticModel::FieldComponents(real Bx, real By, real Bz,
                                      real Bxt, real Byt, real Bzt,
                                      real& H, real& F, real& D, real& I,
                                      real& Ht, real& Ft,
                                      real& Dt, real& It) {
    H = hypot(Bx, By);
    Ht = H != 0 ? (Bx * Bxt + By * Byt) / H : hypot(Bxt, Byt);
    D = H != 0 ? Math::atan2d(Bx, By) : Math::atan2d(Bxt, Byt);
    Dt = (H != 0 ? (By * Bxt - Bx * Byt) / Math::sq(H) : 0) / Math::degree();
    F = hypot(H, Bz);
    Ft = F != 0 ? (H * Ht + Bz * Bzt) / F : hypot(Ht, Bzt);
    I = F != 0 ? Math::atan2d(-Bz, H) : Math::atan2d(-Bzt, Ht);
    It = (F != 0 ? (Bz * Ht - H * Bzt) / Math::sq(F) : 0) / Math::degree();
  }

  string MagneticModel::DefaultMagneticPath() {
    string path;
    char* magneticpath = getenv("GEOGRAPHICLIB_MAGNETIC_PATH");
    if (magneticpath)
      path = string(magneticpath);
    if (!path.empty())
      return path;
    char* datapath = getenv("GEOGRAPHICLIB_DATA");
    if (datapath)
      path = string(datapath);
    return (!path.empty() ? path : string(GEOGRAPHICLIB_DATA)) + "/magnetic";
  }

  string MagneticModel::DefaultMagneticName() {
    string name;
    char* magneticname = getenv("GEOGRAPHICLIB_MAGNETIC_NAME");
    if (magneticname)
      name = string(magneticname);
    return !name.empty() ? name : string(GEOGRAPHICLIB_MAGNETIC_DEFAULT_NAME);
  }

} // namespace GeographicLib
