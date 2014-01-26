/**
 * \file mgrsforward.cpp
 * \brief Matlab mex file for UTM/UPS to MGRS conversions
 *
 * Copyright (c) Charles Karney (2010) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

// Compile in Matlab with
// [Unix]
// mex -I/usr/local/include -L/usr/local/lib -Wl,-rpath=/usr/local/lib
//    -lGeographic mgrsforward.cpp
// [Windows]
// mex -I../include -L../windows/Release
//    -lGeographic mgrsforward.cpp

#include <GeographicLib/MGRS.hpp>
#include <mex.h>

using namespace std;
using namespace GeographicLib;

void mexFunction( int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[] ) {

  if (nrhs < 1)
    mexErrMsgTxt("One input argument required.");
  else if (nrhs > 2)
    mexErrMsgTxt("More than two input arguments specified.");
  else if (nlhs > 1)
    mexErrMsgTxt("Only one output argument can be specified.");

  if (!mxIsDouble(prhs[0]))
    mexErrMsgTxt("utmups coordinates are not of type double.");

  if (mxGetN(prhs[0]) != 4)
    mexErrMsgTxt("utmups coordinates must be M x 4 matrix.");

  int prec;
  if (nrhs == 1)
    prec = 5;
  else {
    if (!( mxIsDouble(prhs[1]) && !mxIsComplex(prhs[1]) &&
           mxGetNumberOfElements(prhs[1]) == 1 ))
      mexErrMsgTxt("Precision is not an integer.");
    double rprec = mxGetScalar(prhs[1]);
    prec = int(rprec);
    if (double(prec) != rprec)
      mexErrMsgTxt("Precision is not an integer.");
    if (prec < 0 || prec > 11)
      mexErrMsgTxt("Precision outside the legal range [0, 11].");
  }

  mwSize m = mxGetM(prhs[0]);

  double* x = mxGetPr(prhs[0]);
  double* y = x + m;
  double* zone = x + 2*m;
  double* hemi = x + 3*m;

  string mgrsstr;
  plhs[0] = mxCreateCellArray(1, &m);
  mxArray* mgrs = plhs[0];

  for (mwIndex i = 0; i < m; ++i) {
    try {
      mgrsstr = "INVALID";
      int ZONE = int(zone[i]);
      if (double(ZONE) != zone[i])
        throw GeographicErr("Zone is not an integer");
      bool HEMI = (hemi[i] != 0);
      if (HEMI && (hemi[i] != 1))
        throw GeographicErr("Hemisphere is not 0 or 1");
      MGRS::Forward(ZONE, HEMI, x[i], y[i], prec, mgrsstr);
    }
    catch (const std::exception& e) {
      mexWarnMsgTxt(e.what());
    }
    mxSetCell(mgrs, i, mxCreateString(mgrsstr.c_str()));
  }
}
