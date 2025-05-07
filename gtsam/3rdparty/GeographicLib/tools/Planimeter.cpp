/**
 * \file Planimeter.cpp
 * \brief Command line utility for measuring the area of geodesic polygons
 *
 * Copyright (c) Charles Karney (2010-2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * See the <a href="Planimeter.1.html">man page</a> for usage information.
 **********************************************************************/

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <GeographicLib/PolygonArea.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/AuxLatitude.hpp>

#include "Planimeter.usage"

int main(int argc, const char* const argv[]) {
  try {
    using namespace GeographicLib;
    typedef Math::real real;
    Utility::set_digits();
    enum { GEODESIC, AUTHALIC, RHUMB };
    real
      a = Constants::WGS84_a(),
      f = Constants::WGS84_f();
    bool reverse = false, sign = true, polyline = false, longfirst = false,
      exact = false, geoconvert_compat = false;
    int linetype = GEODESIC;
    int prec = 6;
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';';

    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-r")
        reverse = !reverse;
      else if (arg == "-s")
        sign = !sign;
      else if (arg == "-l")
        polyline = !polyline;
      else if (arg == "-e") {
        if (m + 2 >= argc) return usage(1, true);
        try {
          a = Utility::val<real>(std::string(argv[m + 1]));
          f = Utility::fract<real>(std::string(argv[m + 2]));
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding arguments of -e: " << e.what() << "\n";
          return 1;
        }
        m += 2;
      } else if (arg == "-w")
        longfirst = !longfirst;
      else if (arg == "-p") {
        if (++m == argc) return usage(1, true);
        try {
          prec = Utility::val<int>(std::string(argv[m]));
        }
        catch (const std::exception&) {
          std::cerr << "Precision " << argv[m] << " is not a number\n";
          return 1;
        }
      } else if (arg == "-G")
        linetype = GEODESIC;
      else if (arg == "-Q")
        linetype = AUTHALIC;
      else if (arg == "-R")
        linetype = RHUMB;
      else if (arg == "-E")
        exact = true;
      else if (arg == "--geoconvert-input")
        geoconvert_compat = true;
      else if (arg == "--input-string") {
        if (++m == argc) return usage(1, true);
        istring = argv[m];
      } else if (arg == "--input-file") {
        if (++m == argc) return usage(1, true);
        ifile = argv[m];
      } else if (arg == "--output-file") {
        if (++m == argc) return usage(1, true);
        ofile = argv[m];
      } else if (arg == "--line-separator") {
        if (++m == argc) return usage(1, true);
        if (std::string(argv[m]).size() != 1) {
          std::cerr << "Line separator must be a single character\n";
          return 1;
        }
        lsep = argv[m][0];
      } else if (arg == "--comment-delimiter") {
        if (++m == argc) return usage(1, true);
        cdelim = argv[m];
      } else if (arg == "--version") {
        std::cout << argv[0] << ": GeographicLib version "
                  << GEOGRAPHICLIB_VERSION_STRING << "\n";
        return 0;
      } else
        return usage(!(arg == "-h" || arg == "--help"), arg != "--help");
    }

    if (!ifile.empty() && !istring.empty()) {
      std::cerr << "Cannot specify --input-string and --input-file together\n";
      return 1;
    }
    if (ifile == "-") ifile.clear();
    std::ifstream infile;
    std::istringstream instring;
    if (!ifile.empty()) {
      infile.open(ifile.c_str());
      if (!infile.is_open()) {
        std::cerr << "Cannot open " << ifile << " for reading\n";
        return 1;
      }
    } else if (!istring.empty()) {
      std::string::size_type m = 0;
      while (true) {
        m = istring.find(lsep, m);
        if (m == std::string::npos)
          break;
        istring[m] = '\n';
      }
      instring.str(istring);
    }
    std::istream* input = !ifile.empty() ? &infile :
      (!istring.empty() ? &instring : &std::cin);

    std::ofstream outfile;
    if (ofile == "-") ofile.clear();
    if (!ofile.empty()) {
      outfile.open(ofile.c_str());
      if (!outfile.is_open()) {
        std::cerr << "Cannot open " << ofile << " for writing\n";
        return 1;
      }
    }
    std::ostream* output = !ofile.empty() ? &outfile : &std::cout;

    // Linetype is one of GEODESIC, AUTHALIC, RHUMB
    const AuxLatitude ellip(a, f);
    if (linetype == AUTHALIC) {
      // adjusting a and f to correspond to the authalic sphere
      using std::sqrt;
      a = sqrt(ellip.AuthalicRadiusSquared(exact));
      f = 0;
    }
    const Geodesic geod(a, linetype != RHUMB ? f : 0,
                        exact && linetype != RHUMB);
    const Rhumb rhumb(a, linetype == RHUMB ? f : 0,
                      exact && linetype == RHUMB);
    PolygonArea poly(geod, polyline);
    PolygonAreaRhumb polyr(rhumb, polyline);
    GeoCoords p;

    // Max precision = 10: 0.1 nm in distance, 10^-15 deg (= 0.11 nm),
    // 10^-11 sec (= 0.3 nm).
    prec = std::min(10 + Math::extra_digits(), std::max(0, prec));
    std::string s, eol("\n");
    real perimeter, area;
    unsigned num;
    std::istringstream str;
    std::string slat, slon, junk;
    real lat = 0, lon = 0;
    while (std::getline(*input, s)) {
      if (!cdelim.empty()) {
        std::string::size_type m = s.find(cdelim);
        if (m != std::string::npos) {
          eol = " " + s.substr(m) + "\n";
          s = s.substr(0, m);
        }
      }
      bool endpoly = s.empty();
      if (!endpoly) {
        try {
          using std::isnan;
          if (geoconvert_compat) {
            p.Reset(s, true, longfirst);
            lat = p.Latitude(); lon = p.Longitude();
          } else {
            str.clear(); str.str(s);
            if (!(str >> slat >> slon))
              throw GeographicErr("incomplete input");
            if (str >> junk)
              throw GeographicErr("extra input");
            DMS::DecodeLatLon(slat, slon, lat, lon, longfirst);
          }
          if (isnan(lat) || isnan(lon))
            endpoly = true;
        }
        catch (const GeographicErr&) {
          endpoly = true;
        }
      }
      if (endpoly) {
        num =
          linetype == RHUMB ? polyr.Compute(reverse, sign, perimeter, area) :
          poly.Compute(reverse, sign, perimeter, area); // geodesic + authalic
        if (num > 0) {
          *output << num << " " << Utility::str(perimeter, prec);
          if (!polyline) {
            *output << " " << Utility::str(area, std::max(0, prec - 5));
          }
          *output << eol;
        }
        linetype == RHUMB ? polyr.Clear() : poly.Clear();
        eol = "\n";
      } else {
        linetype == RHUMB ? polyr.AddPoint(lat, lon) :
          poly.AddPoint
          (linetype == AUTHALIC ?
           ellip.Convert(AuxLatitude::PHI, AuxLatitude::XI, lat, exact) : lat,
           lon);
      }
    }
    num =
      linetype == RHUMB ? polyr.Compute(reverse, sign, perimeter, area) :
      poly.Compute(reverse, sign, perimeter, area);
    if (num > 0) {
      *output << num << " " << Utility::str(perimeter, prec);
      if (!polyline) {
        *output << " " << Utility::str(area, std::max(0, prec - 5));
      }
      *output << eol;
    }
      linetype == RHUMB ? polyr.Clear() : poly.Clear();
    eol = "\n";
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
  catch (...) {
    std::cerr << "Caught unknown exception\n";
    return 1;
  }
}
