/**
 * \file GravityModel.cpp
 * \brief Implementation for GeographicLib::GravityModel class
 *
 * Copyright (c) Charles Karney (2011-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/GravityModel.hpp>
#include <fstream>
#include <GeographicLib/SphericalEngine.hpp>
#include <GeographicLib/GravityCircle.hpp>
#include <GeographicLib/Utility.hpp>

#if !defined(GEOGRAPHICLIB_DATA)
#  if defined(_WIN32)
#    define GEOGRAPHICLIB_DATA "C:/ProgramData/GeographicLib"
#  else
#    define GEOGRAPHICLIB_DATA "/usr/local/share/GeographicLib"
#  endif
#endif

#if !defined(GEOGRAPHICLIB_GRAVITY_DEFAULT_NAME)
#  define GEOGRAPHICLIB_GRAVITY_DEFAULT_NAME "egm96"
#endif

#if defined(_MSC_VER)
// Squelch warnings about unsafe use of getenv
#  pragma warning (disable: 4996)
#endif

namespace GeographicLib {

  using namespace std;

  GravityModel::GravityModel(const std::string& name,const std::string& path)
    : _name(name)
    , _dir(path)
    , _description("NONE")
    , _date("UNKNOWN")
    , _amodel(Math::NaN())
    , _GMmodel(Math::NaN())
    , _zeta0(0)
    , _corrmult(1)
    , _norm(SphericalHarmonic::FULL)
  {
    if (_dir.empty())
      _dir = DefaultGravityPath();
    ReadMetadata(_name);
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
      int N, M;
      SphericalEngine::coeff::readcoeffs(coeffstr, N, M, _Cx, _Sx);
      if (!(N >= 0 && M >= 0))
        throw GeographicErr("Degree and order must be at least 0");
      if (_Cx[0] != 0)
        throw GeographicErr("A degree 0 term should be zero");
      _Cx[0] = 1;               // Include the 1/r term in the sum
      _gravitational = SphericalHarmonic(_Cx, _Sx, N, N, M, _amodel, _norm);
      SphericalEngine::coeff::readcoeffs(coeffstr, N, M, _CC, _CS);
      if (N < 0) {
        N = M = 0;
        _CC.resize(1, real(0));
      }
      _CC[0] += _zeta0 / _corrmult;
      _correction = SphericalHarmonic(_CC, _CS, N, N, M, real(1), _norm);
      int pos = int(coeffstr.tellg());
      coeffstr.seekg(0, ios::end);
      if (pos != coeffstr.tellg())
        throw GeographicErr("Extra data in " + coeff);
    }
    int nmx = _gravitational.Coefficients().nmx();
    // Adjust the normalization of the normal potential to match the model.
    real mult = _earth._GM / _GMmodel;
    real amult = Math::sq(_earth._a / _amodel);
    // The 0th term in _zonal should be is 1 + _dzonal0.  Instead set it to 1
    // to give exact cancellation with the (0,0) term in the model and account
    // for _dzonal0 separately.
    _zonal.clear(); _zonal.push_back(1);
    _dzonal0 = (_earth.MassConstant() - _GMmodel) / _GMmodel;
    for (int n = 2; n <= nmx; n += 2) {
      // Only include as many normal zonal terms as matter.  Figuring the limit
      // in this way works because the coefficients of the normal potential
      // (which is smooth) decay much more rapidly that the corresponding
      // coefficient of the model potential (which is bumpy).  Typically this
      // goes out to n = 18.
      mult *= amult;
      real
        r = _Cx[n],                                        // the model term
        s = - mult * _earth.Jn(n) / sqrt(real(2 * n + 1)), // the normal term
        t = r - s;                                         // the difference
      if (t == r)               // the normal term is negligible
        break;
      _zonal.push_back(0);      // index = n - 1; the odd terms are 0
      _zonal.push_back(s);
    }
    int nmx1 = int(_zonal.size()) - 1;
    _disturbing = SphericalHarmonic1(_Cx, _Sx,
                                     _gravitational.Coefficients().N(),
                                     nmx, _gravitational.Coefficients().mmx(),
                                     _zonal,
                                     _zonal, // This is not accessed!
                                     nmx1, nmx1, 0,
                                     _amodel,
                                     SphericalHarmonic1::normalization(_norm));
  }

  void GravityModel::ReadMetadata(const std::string& name) {
    const char* spaces = " \t\n\v\f\r";
    _filename = _dir + "/" + name + ".egm";
    ifstream metastr(_filename.c_str());
    if (!metastr.good())
      throw GeographicErr("Cannot open " + _filename);
    string line;
    getline(metastr, line);
    if (!(line.size() >= 6 && line.substr(0,5) == "EGMF-"))
      throw GeographicErr(_filename + " does not contain EGMF-n signature");
    string::size_type n = line.find_first_of(spaces, 5);
    if (n != string::npos)
      n -= 5;
    string version(line, 5, n);
    if (version != "1")
      throw GeographicErr("Unknown version in " + _filename + ": " + version);
    string key, val;
    real a = Math::NaN(), GM = a, omega = a, f = a, J2 = a;
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
      else if (key == "ModelRadius")
        _amodel = Utility::val<real>(val);
      else if (key == "ModelMass")
        _GMmodel = Utility::val<real>(val);
      else if (key == "AngularVelocity")
        omega = Utility::val<real>(val);
      else if (key == "ReferenceRadius")
        a = Utility::val<real>(val);
      else if (key == "ReferenceMass")
        GM = Utility::val<real>(val);
      else if (key == "Flattening")
        f = Utility::fract<real>(val);
      else if (key == "DynamicalFormFactor")
        J2 = Utility::fract<real>(val);
      else if (key == "HeightOffset")
        _zeta0 = Utility::fract<real>(val);
      else if (key == "CorrectionMultiplier")
        _corrmult = Utility::fract<real>(val);
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
    if (!(Math::isfinite(_amodel) && _amodel > 0))
      throw GeographicErr("Model radius must be positive");
    if (!(Math::isfinite(_GMmodel) && _GMmodel > 0))
      throw GeographicErr("Model mass constant must be positive");
    if (!(Math::isfinite(_corrmult) && _corrmult > 0))
      throw GeographicErr("Correction multiplier must be positive");
    if (!(Math::isfinite(_zeta0)))
      throw GeographicErr("Height offset must be finite");
    if (int(_id.size()) != idlength_)
      throw GeographicErr("Invalid ID");
    if (Math::isfinite(f) && Math::isfinite(J2))
      throw GeographicErr("Cannot specify both f and J2");
    _earth = NormalGravity(a, GM, omega,
                           Math::isfinite(f) ? f : J2, Math::isfinite(f));
  }

  Math::real GravityModel::InternalT(real X, real Y, real Z,
                                     real& deltaX, real& deltaY, real& deltaZ,
                                     bool gradp, bool correct) const {
    // If correct, then produce the correct T = W - U.  Otherwise, neglect the
    // n = 0 term (which is proportial to the difference in the model and
    // reference values of GM).
    if (_dzonal0 == 0)
      // No need to do the correction
      correct = false;
    real T, invR = correct ? 1 / Math::hypot(Math::hypot(X, Y), Z) : 1;
    if (gradp) {
      // initial values to suppress warnings
      deltaX = deltaY = deltaZ = 0;
      T = _disturbing(-1, X, Y, Z, deltaX, deltaY, deltaZ);
      real f = _GMmodel / _amodel;
      deltaX *= f;
      deltaY *= f;
      deltaZ *= f;
      if (correct) {
        invR = _GMmodel * _dzonal0 * invR * invR * invR;
        deltaX += X * invR;
        deltaY += Y * invR;
        deltaZ += Z * invR;
      }
    } else
      T = _disturbing(-1, X, Y, Z);
    T = (T / _amodel - (correct ? _dzonal0 : 0) * invR) * _GMmodel;
    return T;
  }

  Math::real GravityModel::V(real X, real Y, real Z,
                             real& GX, real& GY, real& GZ) const {
    real
      Vres = _gravitational(X, Y, Z, GX, GY, GZ),
      f = _GMmodel / _amodel;
    Vres *= f;
    GX *= f;
    GY *= f;
    GZ *= f;
    return Vres;
  }

  Math::real GravityModel::W(real X, real Y, real Z,
                             real& gX, real& gY, real& gZ) const {
    real fX, fY,
      Wres = V(X, Y, Z, gX, gY, gZ) + _earth.Phi(X, Y, fX, fY);
    gX += fX;
    gY += fY;
    return Wres;
  }

  void GravityModel::SphericalAnomaly(real lat, real lon, real h,
                                      real& Dg01, real& xi, real& eta) const {
    real X, Y, Z, M[Geocentric::dim2_];
    _earth.Earth().IntForward(lat, lon, h, X, Y, Z, M);
    real
      deltax, deltay, deltaz,
      T = InternalT(X, Y, Z, deltax, deltay, deltaz, true, false),
      clam = M[3], slam = -M[0],
      P = Math::hypot(X, Y),
      R = Math::hypot(P, Z),
      // psi is geocentric latitude
      cpsi = R != 0 ? P / R : M[7],
      spsi = R != 0 ? Z / R : M[8];
    // Rotate cartesian into spherical coordinates
    real MC[Geocentric::dim2_];
    Geocentric::Rotation(spsi, cpsi, slam, clam, MC);
    Geocentric::Unrotate(MC, deltax, deltay, deltaz, deltax, deltay, deltaz);
    // H+M, Eq 2-151c
    Dg01 = - deltaz - 2 * T / R;
    real gammaX, gammaY, gammaZ;
    _earth.U(X, Y, Z, gammaX, gammaY, gammaZ);
    real gamma = Math::hypot( Math::hypot(gammaX, gammaY), gammaZ);
    xi  = -(deltay/gamma) / Math::degree();
    eta = -(deltax/gamma) / Math::degree();
  }

  Math::real GravityModel::GeoidHeight(real lat, real lon) const
  {
    real X, Y, Z;
    _earth.Earth().IntForward(lat, lon, 0, X, Y, Z, NULL);
    real
      gamma0 = _earth.SurfaceGravity(lat),
      dummy,
      T = InternalT(X, Y, Z, dummy, dummy, dummy, false, false),
      invR = 1 / Math::hypot(Math::hypot(X, Y), Z),
      correction = _corrmult * _correction(invR * X, invR * Y, invR * Z);
    // _zeta0 has been included in _correction
    return T/gamma0 + correction;
  }

  Math::real GravityModel::Gravity(real lat, real lon, real h,
                                   real& gx, real& gy, real& gz) const {
    real X, Y, Z, M[Geocentric::dim2_];
    _earth.Earth().IntForward(lat, lon, h, X, Y, Z, M);
    real Wres = W(X, Y, Z, gx, gy, gz);
    Geocentric::Unrotate(M, gx, gy, gz, gx, gy, gz);
    return Wres;
  }
  Math::real GravityModel::Disturbance(real lat, real lon, real h,
                                       real& deltax, real& deltay,
                                       real& deltaz) const {
    real X, Y, Z, M[Geocentric::dim2_];
    _earth.Earth().IntForward(lat, lon, h, X, Y, Z, M);
    real Tres = InternalT(X, Y, Z, deltax, deltay, deltaz, true, true);
    Geocentric::Unrotate(M, deltax, deltay, deltaz, deltax, deltay, deltaz);
    return Tres;
  }

  GravityCircle GravityModel::Circle(real lat, real h, unsigned caps) const {
    if (h != 0)
      // Disallow invoking GeoidHeight unless h is zero.
      caps &= ~(CAP_GAMMA0 | CAP_C);
    real X, Y, Z, M[Geocentric::dim2_];
    _earth.Earth().IntForward(lat, 0, h, X, Y, Z, M);
    // Y = 0, cphi = M[7], sphi = M[8];
    real
      invR = 1 / Math::hypot(X, Z),
      gamma0 = (caps & CAP_GAMMA0 ?_earth.SurfaceGravity(lat)
                : Math::NaN()),
      fx, fy, fz, gamma;
    if (caps & CAP_GAMMA) {
      _earth.U(X, Y, Z, fx, fy, fz); // fy = 0
      gamma = Math::hypot(fx, fz);
    } else
      gamma = Math::NaN();
    _earth.Phi(X, Y, fx, fy);
    return GravityCircle(GravityCircle::mask(caps),
                         _earth._a, _earth._f, lat, h, Z, X, M[7], M[8],
                         _amodel, _GMmodel, _dzonal0, _corrmult,
                         gamma0, gamma, fx,
                         caps & CAP_G ?
                         _gravitational.Circle(X, Z, true) :
                         CircularEngine(),
                         // N.B. If CAP_DELTA is set then CAP_T should be too.
                         caps & CAP_T ?
                         _disturbing.Circle(-1, X, Z, (caps&CAP_DELTA) != 0) :
                         CircularEngine(),
                         caps & CAP_C ?
                         _correction.Circle(invR * X, invR * Z, false) :
                         CircularEngine());
  }

  std::string GravityModel::DefaultGravityPath() {
    string path;
    char* gravitypath = getenv("GEOGRAPHICLIB_GRAVITY_PATH");
    if (gravitypath)
      path = string(gravitypath);
    if (!path.empty())
      return path;
    char* datapath = getenv("GEOGRAPHICLIB_DATA");
    if (datapath)
      path = string(datapath);
    return (!path.empty() ? path : string(GEOGRAPHICLIB_DATA)) + "/gravity";
  }

  std::string GravityModel::DefaultGravityName() {
    string name;
    char* gravityname = getenv("GEOGRAPHICLIB_GRAVITY_NAME");
    if (gravityname)
      name = string(gravityname);
    return !name.empty() ? name : string(GEOGRAPHICLIB_GRAVITY_DEFAULT_NAME);
  }

} // namespace GeographicLib
