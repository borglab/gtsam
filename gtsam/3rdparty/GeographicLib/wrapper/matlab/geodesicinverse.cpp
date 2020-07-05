/**
 * \file geodesicinverse.cpp
 * \brief Matlab mex file for geographic to UTM/UPS conversions
 *
 * Copyright (c) Charles Karney (2010-2013) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

// Compile in Matlab with
// [Unix]
// mex -I/usr/local/include -L/usr/local/lib -Wl,-rpath=/usr/local/lib
//    -lGeographic geodesicinverse.cpp
// [Windows]
// mex -I../include -L../windows/Release
//    -lGeographic geodesicinverse.cpp

#include <algorithm>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <mex.h>

using namespace std;
using namespace GeographicLib;

template<class G> void
compute(double a, double f, mwSize m, const double* latlong,
        double* geodesic, double* aux) {
  const double* lat1 = latlong;
  const double* lon1 = latlong + m;
  const double* lat2 = latlong + 2*m;
  const double* lon2 = latlong + 3*m;
  double* azi1 = geodesic;
  double* azi2 = geodesic + m;
  double* s12 = geodesic + 2*m;
  double* a12 = NULL;
  double* m12 = NULL;
  double* M12 = NULL;
  double* M21 = NULL;
  double* S12 = NULL;
  if (aux) {
    a12 = aux;
    m12 = aux + m;
    M12 = aux + 2*m;
    M21 = aux + 3*m;
    S12 = aux + 4*m;
  }

  const G g(a, f);
  for (mwIndex i = 0; i < m; ++i) {
    if (abs(lat1[i]) <= 90 && lon1[i] >= -540 && lon1[i] < 540 &&
        abs(lat2[i]) <= 90 && lon2[i] >= -540 && lon2[i] < 540) {
      if (aux)
        a12[i] = g.Inverse(lat1[i], lon1[i], lat2[i], lon2[i],
                           s12[i], azi1[i], azi2[i],
                           m12[i], M12[i], M21[i], S12[i]);
      else
        g.Inverse(lat1[i], lon1[i], lat2[i], lon2[i],
                  s12[i], azi1[i], azi2[i]);
    }
  }
}

void mexFunction( int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[] ) {

  if (nrhs < 1)
    mexErrMsgTxt("One input argument required.");
  else if (nrhs > 3)
    mexErrMsgTxt("More than three input arguments specified.");
  else if (nrhs == 2)
    mexErrMsgTxt("Must specify flattening with the equatorial radius.");
  else if (nlhs > 2)
    mexErrMsgTxt("More than two output arguments specified.");

  if (!( mxIsDouble(prhs[0]) && !mxIsComplex(prhs[0]) ))
    mexErrMsgTxt("latlong coordinates are not of type double.");

  if (mxGetN(prhs[0]) != 4)
    mexErrMsgTxt("latlong coordinates must be M x 4 matrix.");

  double a = Constants::WGS84_a<double>(), f = Constants::WGS84_f<double>();
  if (nrhs == 3) {
    if (!( mxIsDouble(prhs[1]) && !mxIsComplex(prhs[1]) &&
           mxGetNumberOfElements(prhs[1]) == 1 ))
      mexErrMsgTxt("Equatorial radius is not a real scalar.");
    a = mxGetScalar(prhs[1]);
    if (!( mxIsDouble(prhs[2]) && !mxIsComplex(prhs[2]) &&
           mxGetNumberOfElements(prhs[2]) == 1 ))
      mexErrMsgTxt("Flattening is not a real scalar.");
    f = mxGetScalar(prhs[2]);
  }

  mwSize m = mxGetM(prhs[0]);

  const double* latlong = mxGetPr(prhs[0]);

  double* geodesic = mxGetPr(plhs[0] = mxCreateDoubleMatrix(m, 3, mxREAL));
  std::fill(geodesic, geodesic + 3*m, Math::NaN<double>());

  double* aux =
    nlhs == 2 ? mxGetPr(plhs[1] = mxCreateDoubleMatrix(m, 5, mxREAL)) :
    NULL;
  if (aux)
    std::fill(aux, aux + 5*m, Math::NaN<double>());

  try {
    if (std::abs(f) <= 0.02)
      compute<Geodesic>(a, f, m, latlong, geodesic, aux);
    else
      compute<GeodesicExact>(a, f, m, latlong, geodesic, aux);
  }
  catch (const std::exception& e) {
    mexErrMsgTxt(e.what());
  }
}
