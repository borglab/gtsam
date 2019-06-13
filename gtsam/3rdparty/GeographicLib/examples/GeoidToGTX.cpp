// Write out a gtx file of geoid heights above the ellipsoid.  For egm2008 at
// 1' resolution this takes about 40 mins on a 8-processor Intel 2.66 GHz
// machine using OpenMP.
//
// For the format of gtx files, see
// http://vdatum.noaa.gov/dev/gtx_info.html#dev_gtx_binary
//
// data is binary big-endian:
//   south latitude edge (degrees double)
//   west longitude edge (degrees double)
//   delta latitude (degrees double)
//   delta longitude (degrees double)
//   nlat = number of latitude rows (integer)
//   nlong = number of longitude columns (integer)
//   nlat * nlong geoid heights (meters float)

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

#if defined(_OPENMP)
#define HAVE_OPENMP 1
#else
#define HAVE_OPENMP 0
#endif

#if HAVE_OPENMP
#  include <omp.h>
#endif

#include <GeographicLib/GravityModel.hpp>
#include <GeographicLib/GravityCircle.hpp>
#include <GeographicLib/Utility.hpp>

using namespace std;
using namespace GeographicLib;

int main(int argc, const char* const argv[]) {
  // Hardwired for 3 args:
  // 1 = the gravity model (e.g., egm2008)
  // 2 = intervals per degree
  // 3 = output GTX file
  if (argc != 4) {
    cerr << "Usage: " << argv[0]
         << " gravity-model intervals-per-degree output.gtx\n";
    return 1;
  }
  try {
    // Will need to set the precision for each thread, so save return value
    int ndigits = Utility::set_digits();
    string model(argv[1]);
    // Number of intervals per degree
    int ndeg = Utility::val<int>(string(argv[2]));
    string filename(argv[3]);
    GravityModel g(model);
    int
      nlat = 180 * ndeg + 1,
      nlon = 360 * ndeg;
    Math::real
      delta = 1 / Math::real(ndeg), // Grid spacing
      latorg = -90,
      lonorg = -180;
    // Write results as floats in binary mode
    ofstream file(filename.c_str(), ios::binary);

    // Write header
    {
      Math::real transform[] = {latorg, lonorg, delta, delta};
      unsigned sizes[] = {unsigned(nlat), unsigned(nlon)};
      Utility::writearray<double, Math::real, true>(file, transform, 4);
      Utility::writearray<unsigned, unsigned, true>(file, sizes, 2);
    }

    // Compute and store results for nbatch latitudes at a time
    const int nbatch = 64;
    vector< vector<float> > N(nbatch, vector<float>(nlon));

    for (int ilat0 = 0; ilat0 < nlat; ilat0 += nbatch) { // Loop over batches
      int nlat0 = min(nlat, ilat0 + nbatch);

#if HAVE_OPENMP
#  pragma omp parallel for
#endif
      for (int ilat = ilat0; ilat < nlat0; ++ilat) { // Loop over latitudes
        Utility::set_digits(ndigits);                // Set the precision
        Math::real
          lat = latorg + (ilat / ndeg) + delta * (ilat - ndeg * (ilat / ndeg)),
          h = 0;
        GravityCircle c(g.Circle(lat, h, GravityModel::GEOID_HEIGHT));
        for (int ilon = 0; ilon < nlon; ++ilon) { // Loop over longitudes
          Math::real lon = lonorg
            + (ilon / ndeg) + delta * (ilon - ndeg * (ilon / ndeg));
          N[ilat - ilat0][ilon] = float(c.GeoidHeight(lon));
        } // longitude loop
      }   // latitude loop -- end of parallel section

      for (int ilat = ilat0; ilat < nlat0; ++ilat) // write out data
        Utility::writearray<float, float, true>(file, N[ilat - ilat0]);
    } // batch loop
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
  catch (...) {
    cerr << "Caught unknown exception\n";
    return 1;
  }
}
