/**
 * \file utmupsforward.cpp
 * \brief Matlab mex file for geographic to UTM/UPS conversions
 *
 * Copyright (c) Charles Karney (2010) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

// Compile in Matlab with
// [Unix]
// mex -I/usr/local/include -L/usr/local/lib -Wl,-rpath=/usr/local/lib
//    -lGeographic utmupsforward.cpp
// [Windows]
// mex -I../include -L../windows/Release
//    -lGeographic utmupsforward.cpp

#include <GeographicLib/UTMUPS.hpp>
#include <mex.h>

using namespace std;
using namespace GeographicLib;

void mexFunction( int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[] ) {

  if (nrhs < 1)
    mexErrMsgTxt("One input argument required.");
  else if (nrhs > 2)
    mexErrMsgTxt("More than two input arguments specified.");
  else if (nlhs > 2)
    mexErrMsgTxt("More than two output arguments specified.");

  if (!( mxIsDouble(prhs[0]) && !mxIsComplex(prhs[0]) ))
    mexErrMsgTxt("latlong coordinates are not of type double.");

  if (mxGetN(prhs[0]) != 2)
    mexErrMsgTxt("latlong coordinates must be M x 2 matrix.");

  int setzone;
  if (nrhs == 1)
    setzone = UTMUPS::STANDARD;
  else {
    if (!( mxIsDouble(prhs[1]) && !mxIsComplex(prhs[1]) &&
           mxGetNumberOfElements(prhs[1]) == 1 ))
      mexErrMsgTxt("setzone is not an integer.");
    double rzone = mxGetScalar(prhs[1]);
    setzone = int(rzone);
    if (double(setzone) != rzone)
      mexErrMsgTxt("setzone is not an integer.");
    if (setzone < UTMUPS::MINPSEUDOZONE || setzone > UTMUPS::MAXZONE)
      mexErrMsgTxt("setzone outside the legal range.");
  }

  mwSize m = mxGetM(prhs[0]);

  double* lat = mxGetPr(prhs[0]);
  double* lon = lat + m;

  plhs[0] = mxCreateDoubleMatrix(m, 4, mxREAL);
  double* x = mxGetPr(plhs[0]);
  double* y = x + m;
  double* zone = x + 2*m;
  double* hemi = x + 3*m;
  double* gamma = NULL;
  double* k = NULL;
  bool scale = nlhs == 2;

  if (scale) {
    plhs[1] = mxCreateDoubleMatrix(m, 2, mxREAL);
    gamma = mxGetPr(plhs[1]);
    k = gamma + m;
  }

  for (mwIndex i = 0; i < m; ++i) {
    int ZONE;
    bool HEMI;
    try {
      if (scale)
        UTMUPS::Forward(lat[i], lon[i], ZONE, HEMI, x[i], y[i], gamma[i], k[i],
                        setzone);
      else
        UTMUPS::Forward(lat[i], lon[i], ZONE, HEMI, x[i], y[i], setzone);
      zone[i] = ZONE;
      hemi[i] = HEMI ? 1 : 0;
    }
    catch (const std::exception& e) {
      mexWarnMsgTxt(e.what());
      x[i] = y[i] = Math::NaN<double>();
      if (scale) gamma[i] = k[i] = Math::NaN<double>();
      zone[i] = UTMUPS::INVALID; hemi[i] = 0;
    }
  }
}
