// Example of using the GeographicLib::NearestNeighbor class.  WARNING: this
// creates a file, pointset.xml or pointset.txt, in the current directory.
// Read lat/lon locations from locations.txt and lat/lon queries from
// queries.txt.  For each query print to standard output: the index for the
// closest location and the distance to it.  Print statistics to standard error
// at the end.

#include <iostream>
#include <exception>
#include <vector>
#include <fstream>
#include <string>

#if !defined(GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION)
#define GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION 0
#endif

#if GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION
// If Boost serialization is available, use it.
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#endif

#include <GeographicLib/NearestNeighbor.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/DMS.hpp>

using namespace std;
using namespace GeographicLib;

// A structure to hold a geographic coordinate.
struct pos {
  double _lat, _lon;
  pos(double lat = 0, double lon = 0) : _lat(lat), _lon(lon) {}
};

// A class to compute the distance between 2 positions.
class DistanceCalculator {
private:
  Geodesic _geod;
public:
  explicit DistanceCalculator(const Geodesic& geod) : _geod(geod) {}
  double operator() (const pos& a, const pos& b) const {
    double d;
    _geod.Inverse(a._lat, a._lon, b._lat, b._lon, d);
    if ( !(d >= 0) )
      // Catch illegal positions which result in d = NaN
      throw GeographicErr("distance doesn't satisfy d >= 0");
    return d;
  }
};

int main() {
  try {
    // Read in locations
    vector<pos> locs;
    double lat, lon;
    string sa, sb;
    {
      ifstream is("locations.txt");
      if (!is.good())
        throw GeographicErr("locations.txt not readable");
      while (is >> sa >> sb) {
        DMS::DecodeLatLon(sa, sb, lat, lon);
        locs.push_back(pos(lat, lon));
      }
      if (locs.size() == 0)
        throw GeographicErr("need at least one location");
    }

    // Define a distance function object
    DistanceCalculator distance(Geodesic::WGS84());

    // Create NearestNeighbor object
    NearestNeighbor<double, pos, DistanceCalculator> pointset;

    {
      // Used saved object if it is available
#if GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION
      ifstream is("pointset.xml");
      if (is.good()) {
        boost::archive::xml_iarchive ia(is);
        ia >> BOOST_SERIALIZATION_NVP(pointset);
      }
#else
      ifstream is("pointset.txt");
      if (is.good())
        is >> pointset;
#endif
    }
    // Is the saved pointset up-to-date?
    if (pointset.NumPoints() != int(locs.size())) {
      // else initialize it
      pointset.Initialize(locs, distance);
      // and save it
#if GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION
      ofstream os("pointset.xml");
      if (!os.good())
        throw GeographicErr("cannot write to pointset.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(pointset);
#else
      ofstream os("pointset.txt");
      if (!os.good())
        throw GeographicErr("cannot write to pointset.txt");
      os << pointset << "\n";
#endif
    }

    ifstream is("queries.txt");
    double d;
    int count = 0;
    vector<int> k;
    while (is >> sa >> sb) {
      ++count;
      DMS::DecodeLatLon(sa, sb, lat, lon);
      d = pointset.Search(locs, distance, pos(lat, lon), k);
      if (k.size() != 1)
          throw GeographicErr("unexpected number of results");
      cout << k[0] << " " << d << "\n";
    }
    int setupcost, numsearches, searchcost, mincost, maxcost;
    double mean, sd;
    pointset.Statistics(setupcost, numsearches, searchcost,
                        mincost, maxcost, mean, sd);
    int
      totcost = setupcost + searchcost,
      exhaustivecost = count * pointset.NumPoints();
    cerr
      << "Number of distance calculations = " << totcost << "\n"
      << "With an exhaustive search = " << exhaustivecost << "\n"
      << "Ratio = " << double(totcost) / exhaustivecost << "\n"
      << "Efficiency improvement = "
      << 100 * (1 - double(totcost) / exhaustivecost) << "%\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
