/**
 * \file mgrsreverse.cpp
 * \brief Matlab mex file for UTM/UPS to MGRS conversions
 *
 * Copyright (c) Charles Karney (2010) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

// Compile in Matlab with
// [Unix]
// mex -I/usr/local/include -L/usr/local/lib -Wl,-rpath=/usr/local/lib
//    -lGeographic mgrsreverse.cpp
// [Windows]
// mex -I../include -L../windows/Release
//    -lGeographic mgrsreverse.cpp

#include <GeographicLib/MGRS.hpp>
#include <mex.h>

using namespace std;
using namespace GeographicLib;

void mexFunction( int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[] ) {

  if (nrhs != 1)
    mexErrMsgTxt("One input argument required.");
  else if (nlhs > 2)
    mexErrMsgTxt("More than two output arguments specified.");

  if (!mxIsCell(prhs[0]) || mxGetN(prhs[0]) != 1)
    mexErrMsgTxt("mgrs coordinates should be in a M x 1 cell array.");

  mwSize m = mxGetM(prhs[0]);

  plhs[0] = mxCreateDoubleMatrix(m, 4, mxREAL);
  double* x = mxGetPr(plhs[0]);
  double* y = x + m;
  double* zone = x + 2*m;
  double* hemi = x + 3*m;
  double* prec = NULL;
  bool precp = nlhs == 2;

  if (precp) {
    plhs[1] = mxCreateDoubleMatrix(m, 1, mxREAL);
    prec = mxGetPr(plhs[1]);
  }

  string mgrsstr;
  for (mwIndex i = 0; i < m; ++i) {
    try {
      mxArray* mgrs = mxGetCell(prhs[0], i);
      if (!mxIsChar(mgrs) || mxGetM(mgrs) != 1)
        throw GeographicErr("Cell element not a string");
      mwSize n = mxGetN(mgrs);
      mxChar* mgrschar = mxGetChars(mgrs);
      mgrsstr.resize(n);
      for (mwIndex k = 0; k < n; ++k)
        mgrsstr[k] = mgrschar[k];
      int ZONE, PREC;
      bool HEMI;
      MGRS::Reverse(mgrsstr, ZONE, HEMI, x[i], y[i], PREC);
      zone[i] = ZONE;
      hemi[i] = HEMI ? 1 : 0;
      if (precp)
        prec[i] = PREC;
    }
    catch (const std::exception& e) {
      mexWarnMsgTxt(e.what());
      x[i] = y[i] = Math::NaN<double>();
      zone[i] = UTMUPS::INVALID; hemi[i] = 0;
      if (precp) prec[i] = -1;
    }
  }
}
