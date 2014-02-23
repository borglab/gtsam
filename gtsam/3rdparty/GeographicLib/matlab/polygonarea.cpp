/**
 * \file polygonarea.cpp
 * \brief Matlab mex file for computing the area of a geodesicpolygon
 *
 * Copyright (c) Charles Karney (2011) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

// Compile in Matlab with
// [Unix]
// mex -I/usr/local/include -L/usr/local/lib -Wl,-rpath=/usr/local/lib
//    -lGeographic polygonarea.cpp
// [Windows]
// mex -I../include -L../windows/Release
//    -lGeographic polygonarea.cpp

#include <algorithm>
#include <GeographicLib/PolygonArea.hpp>
#include <mex.h>

using namespace std;
using namespace GeographicLib;

void mexFunction( int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[] ) {

  if (nrhs < 1)
    mexErrMsgTxt("One input argument required.");
  else if (nrhs > 3)
    mexErrMsgTxt("More than three input arguments specified.");
  else if (nrhs == 2)
    mexErrMsgTxt("Must specify flattening with the major radius.");
  else if (nlhs > 2)
    mexErrMsgTxt("More than two output arguments specified.");

  if (!( mxIsDouble(prhs[0]) && !mxIsComplex(prhs[0]) ))
    mexErrMsgTxt("latlong coordinates are not of type double.");

  if (mxGetN(prhs[0]) != 2)
    mexErrMsgTxt("latlong coordinates must be M x 2 matrix.");

  double a = Constants::WGS84_a<double>(), f = Constants::WGS84_f<double>();
  if (nrhs == 3) {
    if (!( mxIsDouble(prhs[1]) && !mxIsComplex(prhs[1]) &&
           mxGetNumberOfElements(prhs[1]) == 1 ))
      mexErrMsgTxt("Major radius is not a real scalar.");
    a = mxGetScalar(prhs[1]);
    if (!( mxIsDouble(prhs[2]) && !mxIsComplex(prhs[2]) &&
           mxGetNumberOfElements(prhs[2]) == 1 ))
      mexErrMsgTxt("Flattening is not a real scalar.");
    f = mxGetScalar(prhs[2]);
  }

  mwSize m = mxGetM(prhs[0]);

  double* lat = mxGetPr(prhs[0]);
  double* lon = lat + m;

  plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
  double* area = mxGetPr(plhs[0]);
  double* perimeter = NULL;
  bool perimeterp = nlhs == 2;

  if (perimeterp) {
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
    perimeter = mxGetPr(plhs[1]);
  }

  try {
    const Geodesic g(a, f);
    PolygonArea p(g, false);
    for (mwIndex i = 0; i < m; ++i) {
      if (!(abs(lat[i]) <= 90))
        throw GeographicErr("Invalid latitude");
      if (!(lon[i] >= -540 || lon[i] < 540))
        throw GeographicErr("Invalid longitude");
      p.AddPoint(lat[i], lon[i]);
    }
    double tp, ta;
    p.Compute(false, true, tp, ta);
    *area = ta;
    if (perimeterp) *perimeter = tp;
  }
  catch (const std::exception& e) {
    mexErrMsgTxt(e.what());
  }
}
