/**
 * \file Geoid.cpp
 * \brief Implementation for GeographicLib::Geoid class
 *
 * Copyright (c) Charles Karney (2009-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Geoid.hpp>
// For getenv
#include <cstdlib>
#include <GeographicLib/Utility.hpp>

#if !defined(GEOGRAPHICLIB_DATA)
#  if defined(_WIN32)
#    define GEOGRAPHICLIB_DATA "C:/ProgramData/GeographicLib"
#  else
#    define GEOGRAPHICLIB_DATA "/usr/local/share/GeographicLib"
#  endif
#endif

#if !defined(GEOGRAPHICLIB_GEOID_DEFAULT_NAME)
#  define GEOGRAPHICLIB_GEOID_DEFAULT_NAME "egm96-5"
#endif

#if defined(_MSC_VER)
// Squelch warnings about unsafe use of getenv
#  pragma warning (disable: 4996)
#endif

namespace GeographicLib {

  using namespace std;

  // This is the transfer matrix for a 3rd order fit with a 12-point stencil
  // with weights
  //
  //   \x -1  0  1  2
  //   y
  //  -1   .  1  1  .
  //   0   1  2  2  1
  //   1   1  2  2  1
  //   2   .  1  1  .
  //
  // A algorithm for n-dimensional polynomial fits is described in
  //   F. H. Lesh,
  //   Multi-dimensional least-squares polynomial curve fitting,
  //   CACM 2, 29-30 (1959).
  //
  // Here's the Maxima code to generate this matrix:
  //
  // /* The stencil and the weights */
  // xarr:[
  //     0, 1,
  // -1, 0, 1, 2,
  // -1, 0, 1, 2,
  //     0, 1]$
  // yarr:[
  //   -1,-1,
  // 0, 0, 0, 0,
  // 1, 1, 1, 1,
  //    2, 2]$
  // warr:[
  //    1, 1,
  // 1, 2, 2, 1,
  // 1, 2, 2, 1,
  //    1, 1]$
  //
  // /* [x exponent, y exponent] for cubic fit */
  // pows:[
  // [0,0],
  // [1,0],[0,1],
  // [2,0],[1,1],[0,2],
  // [3,0],[2,1],[1,2],[0,3]]$
  //
  // basisvec(x,y,pows):=map(lambda([ex],(if ex[1]=0 then 1 else x^ex[1])*
  //     (if ex[2]=0 then 1 else y^ex[2])),pows)$
  // addterm(x,y,f,w,pows):=block([a,b,bb:basisvec(x,y,pows)],
  //   a:w*(transpose(bb).bb),
  //   b:(w*f) * bb,
  //   [a,b])$
  //
  // c3row(k):=block([a,b,c,pows:pows,n],
  //   n:length(pows),
  //   a:zeromatrix(n,n),
  //   b:copylist(part(a,1)),
  //   c:[a,b],
  //   for i:1 thru length(xarr) do
  //   c:c+addterm(xarr[i],yarr[i],if i=k then 1 else 0,warr[i],pows),
  //   a:c[1],b:c[2],
  //   part(transpose( a^^-1 . transpose(b)),1))$
  // c3:[]$
  // for k:1 thru length(warr) do c3:endcons(c3row(k),c3)$
  // c3:apply(matrix,c3)$
  // c0:part(ratsimp(
  // genmatrix(yc,1,length(warr)).abs(c3).genmatrix(yd,length(pows),1)),2)$
  // c3:c0*c3$

  const int Geoid::c0_ = 240; // Common denominator
  const int Geoid::c3_[stencilsize_ * nterms_] = {
      9, -18, -88,    0,  96,   90,   0,   0, -60, -20,
     -9,  18,   8,    0, -96,   30,   0,   0,  60, -20,
      9, -88, -18,   90,  96,    0, -20, -60,   0,   0,
    186, -42, -42, -150, -96, -150,  60,  60,  60,  60,
     54, 162, -78,   30, -24,  -90, -60,  60, -60,  60,
     -9, -32,  18,   30,  24,    0,  20, -60,   0,   0,
     -9,   8,  18,   30, -96,    0, -20,  60,   0,   0,
     54, -78, 162,  -90, -24,   30,  60, -60,  60, -60,
    -54,  78,  78,   90, 144,   90, -60, -60, -60, -60,
      9,  -8, -18,  -30, -24,    0,  20,  60,   0,   0,
     -9,  18, -32,    0,  24,   30,   0,   0, -60,  20,
      9, -18,  -8,    0, -24,  -30,   0,   0,  60,  20,
  };

  // Like c3, but with the coeffs of x, x^2, and x^3 constrained to be zero.
  // Use this at the N pole so that the height in independent of the longitude
  // there.
  //
  // Here's the Maxima code to generate this matrix (continued from above).
  //
  // /* figure which terms to exclude so that fit is indep of x at y=0 */
  // mask:part(zeromatrix(1,length(pows)),1)+1$
  // for i:1 thru length(pows) do
  // if pows[i][1]>0 and pows[i][2]=0 then mask[i]:0$
  //
  // /* Same as c3row but with masked pows. */
  // c3nrow(k):=block([a,b,c,powsa:[],n,d,e],
  //   for i:1 thru length(mask) do if mask[i]>0 then
  //   powsa:endcons(pows[i],powsa),
  //   n:length(powsa),
  //   a:zeromatrix(n,n),
  //   b:copylist(part(a,1)),
  //   c:[a,b],
  //   for i:1 thru length(xarr) do
  //   c:c+addterm(xarr[i],yarr[i],if i=k then 1 else 0,warr[i],powsa),
  //   a:c[1],b:c[2],
  //   d:part(transpose( a^^-1 . transpose(b)),1),
  //   e:[],
  //   for i:1 thru length(mask) do
  //   if mask[i]>0 then (e:endcons(first(d),e),d:rest(d)) else e:endcons(0,e),
  //   e)$
  // c3n:[]$
  // for k:1 thru length(warr) do c3n:endcons(c3nrow(k),c3n)$
  // c3n:apply(matrix,c3n)$
  // c0n:part(ratsimp(
  //    genmatrix(yc,1,length(warr)).abs(c3n).genmatrix(yd,length(pows),1)),2)$
  // c3n:c0n*c3n$

  const int Geoid::c0n_ = 372; // Common denominator
  const int Geoid::c3n_[stencilsize_ * nterms_] = {
      0, 0, -131, 0,  138,  144, 0,   0, -102, -31,
      0, 0,    7, 0, -138,   42, 0,   0,  102, -31,
     62, 0,  -31, 0,    0,  -62, 0,   0,    0,  31,
    124, 0,  -62, 0,    0, -124, 0,   0,    0,  62,
    124, 0,  -62, 0,    0, -124, 0,   0,    0,  62,
     62, 0,  -31, 0,    0,  -62, 0,   0,    0,  31,
      0, 0,   45, 0, -183,   -9, 0,  93,   18,   0,
      0, 0,  216, 0,   33,   87, 0, -93,   12, -93,
      0, 0,  156, 0,  153,   99, 0, -93,  -12, -93,
      0, 0,  -45, 0,   -3,    9, 0,  93,  -18,   0,
      0, 0,  -55, 0,   48,   42, 0,   0,  -84,  31,
      0, 0,   -7, 0,  -48,  -42, 0,   0,   84,  31,
  };

  // Like c3n, but y -> 1-y so that h is independent of x at y = 1.  Use this
  // at the S pole so that the height in independent of the longitude there.
  //
  // Here's the Maxima code to generate this matrix (continued from above).
  //
  // /* Transform c3n to c3s by transforming y -> 1-y */
  // vv:[
  //      v[11],v[12],
  // v[7],v[8],v[9],v[10],
  // v[3],v[4],v[5],v[6],
  //      v[1],v[2]]$
  // poly:expand(vv.(c3n/c0n).transpose(basisvec(x,1-y,pows)))$
  // c3sf[i,j]:=coeff(coeff(coeff(poly,v[i]),x,pows[j][1]),y,pows[j][2])$
  // c3s:genmatrix(c3sf,length(vv),length(pows))$
  // c0s:part(ratsimp(
  //    genmatrix(yc,1,length(warr)).abs(c3s).genmatrix(yd,length(pows),1)),2)$
  // c3s:c0s*c3s$

  const int Geoid::c0s_ = 372; // Common denominator
  const int Geoid::c3s_[stencilsize_ * nterms_] = {
     18,  -36, -122,   0,  120,  135, 0,   0,  -84, -31,
    -18,   36,   -2,   0, -120,   51, 0,   0,   84, -31,
     36, -165,  -27,  93,  147,   -9, 0, -93,   18,   0,
    210,   45, -111, -93,  -57, -192, 0,  93,   12,  93,
    162,  141,  -75, -93, -129, -180, 0,  93,  -12,  93,
    -36,  -21,   27,  93,   39,    9, 0, -93,  -18,   0,
      0,    0,   62,   0,    0,   31, 0,   0,    0, -31,
      0,    0,  124,   0,    0,   62, 0,   0,    0, -62,
      0,    0,  124,   0,    0,   62, 0,   0,    0, -62,
      0,    0,   62,   0,    0,   31, 0,   0,    0, -31,
    -18,   36,  -64,   0,   66,   51, 0,   0, -102,  31,
     18,  -36,    2,   0,  -66,  -51, 0,   0,  102,  31,
  };

  Geoid::Geoid(const std::string& name, const std::string& path, bool cubic,
               bool threadsafe)
    : _name(name)
    , _dir(path)
    , _cubic(cubic)
    , _a( Constants::WGS84_a() )
    , _e2( (2 - Constants::WGS84_f()) * Constants::WGS84_f() )
    , _degree( Math::degree() )
    , _eps( sqrt(numeric_limits<real>::epsilon()) )
    , _threadsafe(false)        // Set after cache is read
  {
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(pixel_t) == pixel_size_,
                                "pixel_t has the wrong size");
    if (_dir.empty())
      _dir = DefaultGeoidPath();
    _filename = _dir + "/" + _name + (pixel_size_ != 4 ? ".pgm" : ".pgm4");
    _file.open(_filename.c_str(), ios::binary);
    if (!(_file.good()))
      throw GeographicErr("File not readable " + _filename);
    string s;
    if (!(getline(_file, s) && s == "P5"))
      throw GeographicErr("File not in PGM format " + _filename);
    _offset = numeric_limits<real>::max();
    _scale = 0;
    _maxerror = _rmserror = -1;
    _description = "NONE";
    _datetime = "UNKNOWN";
    while (getline(_file, s)) {
      if (s.empty())
        continue;
      if (s[0] == '#') {
        istringstream is(s);
        string commentid, key;
        if (!(is >> commentid >> key) || commentid != "#")
          continue;
        if (key == "Description" || key =="DateTime") {
          string::size_type p =
            s.find_first_not_of(" \t", unsigned(is.tellg()));
          if (p != string::npos)
            (key == "Description" ? _description : _datetime) = s.substr(p);
        } else if (key == "Offset") {
          if (!(is >> _offset))
            throw GeographicErr("Error reading offset " + _filename);
        } else if (key == "Scale") {
          if (!(is >> _scale))
            throw GeographicErr("Error reading scale " + _filename);
        } else if (key == (_cubic ? "MaxCubicError" : "MaxBilinearError")) {
          // It's not an error if the error can't be read
          is >> _maxerror;
        } else if (key == (_cubic ? "RMSCubicError" : "RMSBilinearError")) {
          // It's not an error if the error can't be read
          is >> _rmserror;
        }
      } else {
        istringstream is(s);
        if (!(is >> _width >> _height))
          throw GeographicErr("Error reading raster size " + _filename);
        break;
      }
    }
    {
      unsigned maxval;
      if (!(_file >> maxval))
        throw GeographicErr("Error reading maxval " + _filename);
      if (maxval != pixel_max_)
        throw GeographicErr("Incorrect value of maxval " + _filename);
      // Add 1 for whitespace after maxval
      _datastart = (unsigned long long)(_file.tellg()) + 1ULL;
      _swidth = (unsigned long long)(_width);
    }
    if (_offset == numeric_limits<real>::max())
      throw GeographicErr("Offset not set " + _filename);
    if (_scale == 0)
      throw GeographicErr("Scale not set " + _filename);
    if (_scale < 0)
      throw GeographicErr("Scale must be positive " + _filename);
    if (_height < 2 || _width < 2)
      // Coarsest grid spacing is 180deg.
      throw GeographicErr("Raster size too small " + _filename);
    if (_width & 1)
      // This is so that longitude grids can be extended thru the poles.
      throw GeographicErr("Raster width is odd " + _filename);
    if (!(_height & 1))
      // This is so that latitude grid includes the equator.
      throw GeographicErr("Raster height is even " + _filename);
    _file.seekg(0, ios::end);
    if (!_file.good() ||
        _datastart + pixel_size_ * _swidth * (unsigned long long)(_height) !=
        (unsigned long long)(_file.tellg()))
      // Possibly this test should be "<" because the file contains, e.g., a
      // second image.  However, for now we are more strict.
      throw GeographicErr("File has the wrong length " + _filename);
    _rlonres = _width / real(360);
    _rlatres = (_height - 1) / real(180);
    _cache = false;
    _ix = _width;
    _iy = _height;
    // Ensure that file errors throw exceptions
    _file.exceptions(ifstream::eofbit | ifstream::failbit | ifstream::badbit);
    if (threadsafe) {
      CacheAll();
      _file.close();
      _threadsafe = true;
    }
  }

  Math::real Geoid::height(real lat, real lon) const {
    lat = Math::LatFix(lat);
    if (Math::isnan(lat) || Math::isnan(lon)) {
      return Math::NaN();
    }
    lon = Math::AngNormalize(lon);
    real
      fx =  lon * _rlonres,
      fy = -lat * _rlatres;
    int
      ix = int(floor(fx)),
      iy = min((_height - 1)/2 - 1, int(floor(fy)));
    fx -= ix;
    fy -= iy;
    iy += (_height - 1)/2;
    ix += ix < 0 ? _width : (ix >= _width ? -_width : 0);
    real v00 = 0, v01 = 0, v10 = 0, v11 = 0;
    real t[nterms_];

    if (_threadsafe || !(ix == _ix && iy == _iy)) {
      if (!_cubic) {
        v00 = rawval(ix    , iy    );
        v01 = rawval(ix + 1, iy    );
        v10 = rawval(ix    , iy + 1);
        v11 = rawval(ix + 1, iy + 1);
      } else {
        real v[stencilsize_];
        int k = 0;
        v[k++] = rawval(ix    , iy - 1);
        v[k++] = rawval(ix + 1, iy - 1);
        v[k++] = rawval(ix - 1, iy    );
        v[k++] = rawval(ix    , iy    );
        v[k++] = rawval(ix + 1, iy    );
        v[k++] = rawval(ix + 2, iy    );
        v[k++] = rawval(ix - 1, iy + 1);
        v[k++] = rawval(ix    , iy + 1);
        v[k++] = rawval(ix + 1, iy + 1);
        v[k++] = rawval(ix + 2, iy + 1);
        v[k++] = rawval(ix    , iy + 2);
        v[k++] = rawval(ix + 1, iy + 2);

        const int* c3x = iy == 0 ? c3n_ : (iy == _height - 2 ? c3s_ : c3_);
        int c0x = iy == 0 ? c0n_ : (iy == _height - 2 ? c0s_ : c0_);
        for (unsigned i = 0; i < nterms_; ++i) {
          t[i] = 0;
          for (unsigned j = 0; j < stencilsize_; ++j)
            t[i] += v[j] * c3x[nterms_ * j + i];
          t[i] /= c0x;
        }
      }
    } else { // same cell; used cached coefficients
      if (!_cubic) {
        v00 = _v00;
        v01 = _v01;
        v10 = _v10;
        v11 = _v11;
      } else
        copy(_t, _t + nterms_, t);
    }
    if (!_cubic) {
      real
        a = (1 - fx) * v00 + fx * v01,
        b = (1 - fx) * v10 + fx * v11,
        c = (1 - fy) * a + fy * b,
        h = _offset + _scale * c;
      if (!_threadsafe) {
        _ix = ix;
        _iy = iy;
        _v00 = v00;
        _v01 = v01;
        _v10 = v10;
        _v11 = v11;
      }
      return h;
    } else {
      real h = t[0] + fx * (t[1] + fx * (t[3] + fx * t[6])) +
        fy * (t[2] + fx * (t[4] + fx * t[7]) +
             fy * (t[5] + fx * t[8] + fy * t[9]));
      h = _offset + _scale * h;
      if (!_threadsafe) {
        _ix = ix;
        _iy = iy;
        copy(t, t + nterms_, _t);
      }
      return h;
    }
  }

  void Geoid::CacheClear() const {
    if (!_threadsafe) {
      _cache = false;
      try {
        _data.clear();
        // Use swap to release memory back to system
        vector< vector<pixel_t> >().swap(_data);
      }
      catch (const exception&) {
      }
    }
  }

  void Geoid::CacheArea(real south, real west, real north, real east) const {
    if (_threadsafe)
      throw GeographicErr("Attempt to change cache of threadsafe Geoid");
    if (south > north) {
      CacheClear();
      return;
    }
    south = Math::LatFix(south);
    north = Math::LatFix(north);
    west = Math::AngNormalize(west); // west in [-180, 180)
    east = Math::AngNormalize(east);
    if (east <= west)
      east += 360;              // east - west in (0, 360]
    int
      iw = int(floor(west * _rlonres)),
      ie = int(floor(east * _rlonres)),
      in = int(floor(-north * _rlatres)) + (_height - 1)/2,
      is = int(floor(-south * _rlatres)) + (_height - 1)/2;
    in = max(0, min(_height - 2, in));
    is = max(0, min(_height - 2, is));
    is += 1;
    ie += 1;
    if (_cubic) {
      in -= 1;
      is += 1;
      iw -= 1;
      ie += 1;
    }
    if (ie - iw >= _width - 1) {
      // Include entire longitude range
      iw = 0;
      ie = _width - 1;
    } else {
      ie += iw < 0 ? _width : (iw >= _width ? -_width : 0);
      iw += iw < 0 ? _width : (iw >= _width ? -_width : 0);
    }
    int oysize = int(_data.size());
    _xsize = ie - iw + 1;
    _ysize = is - in + 1;
    _xoffset = iw;
    _yoffset = in;

    try {
      _data.resize(_ysize, vector<pixel_t>(_xsize));
      for (int iy = min(oysize, _ysize); iy--;)
        _data[iy].resize(_xsize);
    }
    catch (const bad_alloc&) {
      CacheClear();
      throw GeographicErr("Insufficient memory for caching " + _filename);
    }

    try {
      for (int iy = in; iy <= is; ++iy) {
        int iy1 = iy, iw1 = iw;
        if (iy < 0 || iy >= _height) {
          // Allow points "beyond" the poles to support interpolation
          iy1 = iy1 < 0 ? -iy1 : 2 * (_height - 1) - iy1;
          iw1 += _width/2;
          if (iw1 >= _width)
            iw1 -= _width;
        }
        int xs1 = min(_width - iw1, _xsize);
        filepos(iw1, iy1);
        Utility::readarray<pixel_t, pixel_t, true>
          (_file, &(_data[iy - in][0]), xs1);
        if (xs1 < _xsize) {
          // Wrap around longitude = 0
          filepos(0, iy1);
          Utility::readarray<pixel_t, pixel_t, true>
            (_file, &(_data[iy - in][xs1]), _xsize - xs1);
        }
      }
      _cache = true;
    }
    catch (const exception& e) {
      CacheClear();
      throw GeographicErr(string("Error filling cache ") + e.what());
    }
  }

  std::string Geoid::DefaultGeoidPath() {
    string path;
    char* geoidpath = getenv("GEOGRAPHICLIB_GEOID_PATH");
    if (geoidpath)
      path = string(geoidpath);
    if (!path.empty())
      return path;
    char* datapath = getenv("GEOGRAPHICLIB_DATA");
    if (datapath)
      path = string(datapath);
    return (!path.empty() ? path : string(GEOGRAPHICLIB_DATA)) + "/geoids";
  }

  std::string Geoid::DefaultGeoidName() {
    string name;
    char* geoidname = getenv("GEOGRAPHICLIB_GEOID_NAME");
    if (geoidname)
      name = string(geoidname);
    return !name.empty() ? name : string(GEOGRAPHICLIB_GEOID_DEFAULT_NAME);
  }

} // namespace GeographicLib
