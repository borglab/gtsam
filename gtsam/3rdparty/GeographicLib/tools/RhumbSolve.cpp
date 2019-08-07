/**
 * \file RhumbSolve.cpp
 * \brief Command line utility for rhumb line calculations
 *
 * Copyright (c) Charles Karney (2014-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * See the <a href="RhumbSolve.1.html">man page</a> for usage information.
 **********************************************************************/

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <limits>
#include <GeographicLib/Rhumb.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions and potentially
// uninitialized local variables
#  pragma warning (disable: 4127 4701)
#endif

#include "RhumbSolve.usage"

using namespace GeographicLib;
typedef Math::real real;

std::string LatLonString(real lat, real lon, int prec, bool dms, char dmssep,
                         bool longfirst) {
  using namespace GeographicLib;
  std::string
    latstr = dms ? DMS::Encode(lat, prec + 5, DMS::LATITUDE, dmssep) :
    DMS::Encode(lat, prec + 5, DMS::NUMBER),
    lonstr = dms ? DMS::Encode(lon, prec + 5, DMS::LONGITUDE, dmssep) :
    DMS::Encode(lon, prec + 5, DMS::NUMBER);
  return
    (longfirst ? lonstr : latstr) + " " + (longfirst ? latstr : lonstr);
}

std::string AzimuthString(real azi, int prec, bool dms, char dmssep) {
  return dms ? DMS::Encode(azi, prec + 5, DMS::AZIMUTH, dmssep) :
    DMS::Encode(azi, prec + 5, DMS::NUMBER);
}

int main(int argc, const char* const argv[]) {
  try {
    Utility::set_digits();
    bool linecalc = false, inverse = false, dms = false, exact = true,
      longfirst = false;
    real
      a = Constants::WGS84_a(),
      f = Constants::WGS84_f();
    real lat1, lon1, azi12 = Math::NaN(), lat2, lon2, s12, S12;
    int prec = 3;
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';', dmssep = char(0);

    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-i") {
        inverse = true;
        linecalc = false;
      } else if (arg == "-L" || arg == "-l") { // -l is DEPRECATED
        inverse = false;
        linecalc = true;
        if (m + 3 >= argc) return usage(1, true);
        try {
          DMS::DecodeLatLon(std::string(argv[m + 1]), std::string(argv[m + 2]),
                            lat1, lon1, longfirst);
          azi12 = DMS::DecodeAzimuth(std::string(argv[m + 3]));
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding arguments of -L: " << e.what() << "\n";
          return 1;
        }
        m += 3;
      } else if (arg == "-e") {
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
      }
      else if (arg == "-d") {
        dms = true;
        dmssep = '\0';
      } else if (arg == "-:") {
        dms = true;
        dmssep = ':';
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
      } else if (arg == "-s")
        exact = false;
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

    const Rhumb rh(a, f, exact);
    const RhumbLine rhl(linecalc ? rh.Line(lat1, lon1, azi12) :
                        rh.Line(0, 0, 90));
    // Max precision = 10: 0.1 nm in distance, 10^-15 deg (= 0.11 nm),
    // 10^-11 sec (= 0.3 nm).
    prec = std::min(10 + Math::extra_digits(), std::max(0, prec));
    std::string s, eol, slat1, slon1, slat2, slon2, sazi, ss12, strc;
    std::istringstream str;
    int retval = 0;
    while (std::getline(*input, s)) {
      try {
        eol = "\n";
        if (!cdelim.empty()) {
          std::string::size_type m = s.find(cdelim);
          if (m != std::string::npos) {
            eol = " " + s.substr(m) + "\n";
            s = s.substr(0, m);
          }
        }
        str.clear(); str.str(s);
        if (linecalc) {
          if (!(str >> s12))
            throw GeographicErr("Incomplete input: " + s);
          if (str >> strc)
            throw GeographicErr("Extraneous input: " + strc);
          rhl.Position(s12, lat2, lon2, S12);
          *output << LatLonString(lat2, lon2, prec, dms, dmssep, longfirst)
                  << " " << Utility::str(S12, std::max(prec-7, 0)) << eol;
        } else if (inverse) {
          if (!(str >> slat1 >> slon1 >> slat2 >> slon2))
            throw GeographicErr("Incomplete input: " + s);
          if (str >> strc)
            throw GeographicErr("Extraneous input: " + strc);
          DMS::DecodeLatLon(slat1, slon1, lat1, lon1, longfirst);
          DMS::DecodeLatLon(slat2, slon2, lat2, lon2, longfirst);
          rh.Inverse(lat1, lon1, lat2, lon2, s12, azi12, S12);
          *output << AzimuthString(azi12, prec, dms, dmssep) << " "
                  << Utility::str(s12, prec) << " "
                  << Utility::str(S12, std::max(prec-7, 0)) << eol;
        } else {                // direct
          if (!(str >> slat1 >> slon1 >> sazi >> s12))
            throw GeographicErr("Incomplete input: " + s);
          if (str >> strc)
            throw GeographicErr("Extraneous input: " + strc);
          DMS::DecodeLatLon(slat1, slon1, lat1, lon1, longfirst);
          azi12 = DMS::DecodeAzimuth(sazi);
          rh.Direct(lat1, lon1, azi12, s12, lat2, lon2, S12);
          *output << LatLonString(lat2, lon2, prec, dms, dmssep, longfirst)
                  << " " << Utility::str(S12, std::max(prec-7, 0)) << eol;
        }
      }
      catch (const std::exception& e) {
        // Write error message cout so output lines match input lines
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
