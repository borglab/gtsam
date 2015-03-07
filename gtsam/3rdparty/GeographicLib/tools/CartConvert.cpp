/**
 * \file CartConvert.cpp
 * \brief Command line utility for geodetic to cartesian coordinate conversions
 *
 * Copyright (c) Charles Karney (2009-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 *
 * Compile and link with
 *   g++ -g -O3 -I../include -I../man -o CartConvert \
 *       CartConvert.cpp \
 *       ../src/DMS.cpp \
 *       ../src/Geocentric.cpp \
 *       ../src/LocalCartesian.cpp
 *
 * See the <a href="CartConvert.1.html">man page</a> for usage
 * information.
 **********************************************************************/

#include <iostream>
#include <sstream>
#include <string>
#include <sstream>
#include <fstream>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions and potentially
// uninitialized local variables
#  pragma warning (disable: 4127 4701)
#endif

#include "CartConvert.usage"

int main(int argc, char* argv[]) {
  try {
    using namespace GeographicLib;
    typedef Math::real real;
    bool localcartesian = false, reverse = false;
    real
      a = Constants::WGS84_a<real>(),
      f = Constants::WGS84_f<real>();
    real lat0 = 0, lon0 = 0, h0 = 0;
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';';

    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-r")
        reverse = true;
      else if (arg == "-l") {
        localcartesian = true;
        if (m + 3 >= argc) return usage(1, true);
        try {
          DMS::DecodeLatLon(std::string(argv[m + 1]), std::string(argv[m + 2]),
                            lat0, lon0);
          h0 = Utility::num<real>(std::string(argv[m + 3]));
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding arguments of -l: " << e.what() << "\n";
          return 1;
        }
        m += 3;
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

    const Geocentric ec(a, f);
    const LocalCartesian lc(lat0, lon0, h0, ec);

    std::string s;
    int retval = 0;
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
        real lat, lon, h, x, y, z;
        std::string stra, strb, strc;
        if (!(str >> stra >> strb >> strc))
          throw GeographicErr("Incomplete input: " + s);
        if (reverse) {
          x = Utility::num<real>(stra);
          y = Utility::num<real>(strb);
          z = Utility::num<real>(strc);
        } else {
          DMS::DecodeLatLon(stra, strb, lat, lon);
          h = Utility::num<real>(strc);
        }
        std::string strd;
        if (str >> strd)
          throw GeographicErr("Extraneous input: " + strd);
        if (reverse) {
          if (localcartesian)
            lc.Reverse(x, y, z, lat, lon, h);
          else
            ec.Reverse(x, y, z, lat, lon, h);
          *output << Utility::str<real>(lat, 15) << " "
                  << Utility::str<real>(lon, 15) << " "
                  << Utility::str<real>(h, 12) << eol;
        } else {
          if (localcartesian)
            lc.Forward(lat, lon, h, x, y, z);
          else
            ec.Forward(lat, lon, h, x, y, z);
          *output << Utility::str<real>(x, 10) << " "
                  << Utility::str<real>(y, 10) << " "
                  << Utility::str<real>(z, 10) << eol;
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
