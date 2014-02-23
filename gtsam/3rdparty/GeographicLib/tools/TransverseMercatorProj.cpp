/**
 * \file TransverseMercatorProj.cpp
 * \brief Command line utility for transverse Mercator projections
 *
 * Copyright (c) Charles Karney (2008-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 *
 * Compile and link with
 *   g++ -g -O3 -I../include -I../man -o TransverseMercatorProj \
 *       TransverseMercatorProj.cpp \
 *       ../src/DMS.cpp \
 *       ../src/EllipticFunction.cpp \
 *       ../src/TransverseMercator.cpp \
 *       ../src/TransverseMercatorExact.cpp
 *
 * See the <a href="TransverseMercatorProj.1.html">man page</a> for usage
 * information.
 **********************************************************************/

#include <iostream>
#include <sstream>
#include <string>
#include <sstream>
#include <fstream>
#include <GeographicLib/EllipticFunction.hpp>
#include <GeographicLib/TransverseMercatorExact.hpp>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions and potentially
// uninitialized local variables
#  pragma warning (disable: 4127 4701)
#endif

#include "TransverseMercatorProj.usage"

int main(int argc, char* argv[]) {
  try {
    using namespace GeographicLib;
    typedef Math::real real;
    bool exact = true, extended = false, series = false, reverse = false;
    real
      a = Constants::WGS84_a<real>(),
      f = Constants::WGS84_f<real>(),
      k0 = Constants::UTM_k0<real>(),
      lon0 = 0;
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';';

    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-r")
        reverse = true;
      else if (arg == "-t") {
        exact = true;
        extended = true;
        series = false;
      } else if (arg == "-s") {
        exact = false;
        extended = false;
        series = true;
      } else if (arg == "-l") {
        if (++m >= argc) return usage(1, true);
        try {
          DMS::flag ind;
          lon0 = DMS::Decode(std::string(argv[m]), ind);
          if (ind == DMS::LATITUDE)
            throw GeographicErr("Bad hemisphere");
          if (!(lon0 >= -540 && lon0 < 540))
            throw GeographicErr("Bad longitude");
          lon0 = Math::AngNormalize(lon0);
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding argument of " << arg << ": "
                    << e.what() << "\n";
          return 1;
        }
      } else if (arg == "-k") {
        if (++m >= argc) return usage(1, true);
        try {
          k0 = Utility::num<real>(std::string(argv[m]));
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding argument of " << arg << ": "
                    << e.what() << "\n";
          return 1;
        }
      } else if (arg == "-e") {
        if (m + 2 >= argc) return usage(1, true);
        try {
          a = Utility::num<real>(std::string(argv[m + 1]));
          f = Utility::fract<real>(std::string(argv[m + 2]));
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding arguments of -e: " << e.what() << "\n";
          return 1;
        }
        m += 2;
      } else if (arg == "--input-string") {
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
        std::cout
          << argv[0] << ": GeographicLib version "
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

    const TransverseMercator& TMS =
      series ? TransverseMercator(a, f, k0) : TransverseMercator(1, 0, 1);

    const TransverseMercatorExact& TME =
      exact ? TransverseMercatorExact(a, f, k0, extended)
      : TransverseMercatorExact(1, real(0.1), 1, false);

    std::string s;
    int retval = 0;
    std::cout << std::fixed;
    while (std::getline(*input, s)) {
      try {
        std::string eol("\n");
        if (!cdelim.empty()) {
          std::string::size_type m = s.find(cdelim);
          if (m != std::string::npos) {
            eol = " " + s.substr(m) + "\n";
            s = s.substr(0, m);
          }
        }
        std::istringstream str(s);
        real lat, lon, x, y;
        std::string stra, strb;
        if (!(str >> stra >> strb))
          throw GeographicErr("Incomplete input: " + s);
        if (reverse) {
          x = Utility::num<real>(stra);
          y = Utility::num<real>(strb);
        } else
          DMS::DecodeLatLon(stra, strb, lat, lon);
        std::string strc;
        if (str >> strc)
          throw GeographicErr("Extraneous input: " + strc);
        real gamma, k;
        if (reverse) {
          if (series)
            TMS.Reverse(lon0, x, y, lat, lon, gamma, k);
          else
            TME.Reverse(lon0, x, y, lat, lon, gamma, k);
          *output << Utility::str<real>(lat, 15) << " "
                  << Utility::str<real>(lon, 15) << " "
                  << Utility::str<real>(gamma, 16) << " "
                  << Utility::str<real>(k, 16) << eol;
        } else {
          if (series)
            TMS.Forward(lon0, lat, lon, x, y, gamma, k);
          else
            TME.Forward(lon0, lat, lon, x, y, gamma, k);
          *output << Utility::str<real>(x, 10) << " "
                  << Utility::str<real>(y, 10) << " "
                  << Utility::str<real>(gamma, 16) << " "
                  << Utility::str<real>(k, 16) << eol;
        }
      }
      catch (const std::exception& e) {
        *output << "ERROR: " << e.what() << "\n";
        retval = 1;
      }
    }
    return retval;
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
