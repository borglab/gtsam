/**
 * \file geoidheight.cpp
 * \brief Matlab mex file for geographic to UTM/UPS conversions
 *
 * Copyright (c) Charles Karney (2010) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

// Compile in Matlab with
// [Unix]
// mex -I/usr/local/include -L/usr/local/lib -Wl,-rpath=/usr/local/lib
//    -lGeographic geoidheight.cpp
// [Windows]
// mex -I../include -L../windows/Release
//    -lGeographic geoidheight.cpp

#include <string>
#include <algorithm>
#include <GeographicLib/Geoid.hpp>
#include <mex.h>

using namespace std;
using namespace GeographicLib;

void mexFunction( int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[] ) {

  if (nrhs < 1)
    mexErrMsgTxt("One input argument required.");
  else if (nrhs > 3)
    mexErrMsgTxt("More than three input arguments specified.");
  else if (nlhs > 2)
    mexErrMsgTxt("More than two output arguments specified.");

  if (!( mxIsDouble(prhs[0]) && !mxIsComplex(prhs[0]) ))
    mexErrMsgTxt("latlong coordinates are not of type double.");

  if (mxGetN(prhs[0]) != 2)
    mexErrMsgTxt("latlong coordinates must be M x 2 matrix.");

  mwSize m = mxGetM(prhs[0]);

  double* lat = mxGetPr(prhs[0]);
  double* lon = lat + m;

  plhs[0] = mxCreateDoubleMatrix(m, 1, mxREAL);
  double* h = mxGetPr(plhs[0]);
  std::fill(h, h + m, Math::NaN<double>());
  double* gradn = NULL;
  double* grade = NULL;
  bool gradient = nlhs == 2;

  if (gradient) {
    plhs[1] = mxCreateDoubleMatrix(m, 2, mxREAL);
    gradn = mxGetPr(plhs[1]);
    std::fill(gradn, gradn + 2*m, Math::NaN<double>());
    grade = gradn + m;
  }

  string geoidname("egm96-5");
  if (nrhs > 1) {
    if (!mxIsChar(prhs[1]))
      mexErrMsgTxt("geoid name must be a string.");
    if (mxGetM(prhs[1]) == 0)
      mexErrMsgTxt("geoid name cannot be empty.");
    if (mxGetM(prhs[1]) != 1)
      mexErrMsgTxt("geoid name cannot be a vector of strings.");
    mwSize n = mxGetN(prhs[1]);
    if (n < 1)
      mexErrMsgTxt("geoid name cannot be empty.");
    mxChar* ptr = mxGetChars(prhs[1]);
    geoidname.resize(n);
    for (mwIndex i = 0; i < n; ++i)
      geoidname[i] = ptr[i];
  }
  string geoiddir("");
  if (nrhs > 2) {
    if (!mxIsChar(prhs[2]))
      mexErrMsgTxt("geoid directory must be a string.");
    if (mxGetM(prhs[2]) > 1)
      mexErrMsgTxt("geoid directory cannot be a vector of strings.");
    mwSize n = mxGetN(prhs[2]);
    if (n > 0 && mxGetM(prhs[2]) == 1) {
      mxChar* ptr = mxGetChars(prhs[2]);
      geoiddir.resize(n);
      for (mwIndex i = 0; i < n; ++i)
        geoiddir[i] = ptr[i];
    } // else string is empty and do nothing
  }

  try {
    const Geoid g(geoidname, geoiddir);
    for (mwIndex i = 0; i < m; ++i) {
      if (abs(lat[i]) <= 90 && lon[i] >= -540 && lon[i] < 540) {
        // g() can throw an exception, e.g., because of an I/O failure.  Treat
        // this as fatal.
        if (gradient)
          h[i] = g(lat[i], lon[i], gradn[i], grade[i]);
        else
          h[i] = g(lat[i], lon[i]);
      }
    }
  }
  catch (const std::exception& e) {
    mexErrMsgTxt(e.what());
  }
}
