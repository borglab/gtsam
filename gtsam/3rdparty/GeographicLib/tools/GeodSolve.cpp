/**
 * \file GeodSolve.cpp
 * \brief Command line utility for geodesic calculations
 *
 * Copyright (c) Charles Karney (2009-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 *
 * Compile and link with
 *   g++ -g -O3 -I../include -I../man -o GeodSolve \
 *       GeodSolve.cpp \
 *       ../src/DMS.cpp \
 *       ../src/Geodesic.cpp \
 *       ../src/GeodesicLine.cpp
 *
 * See the <a href="GeodSolve.1.html">man page</a> for usage
 * information.
 **********************************************************************/

#include <iostream>
#include <sstream>
#include <string>
#include <sstream>
#include <fstream>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicLine.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/GeodesicLineExact.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions and potentially
// uninitialized local variables
#  pragma warning (disable: 4127 4701)
#endif

#include "GeodSolve.usage"

typedef GeographicLib::Math::real real;

std::string LatLonString(real lat, real lon, int prec, bool dms, char dmssep) {
  using namespace GeographicLib;
  return dms ?
    DMS::Encode(lat, prec + 5, DMS::LATITUDE, dmssep) + " " +
    DMS::Encode(lon, prec + 5, DMS::LONGITUDE, dmssep) :
    DMS::Encode(lat, prec + 5, DMS::NUMBER) + " " +
    DMS::Encode(lon, prec + 5, DMS::NUMBER);
}

std::string AzimuthString(real azi, int prec, bool dms, char dmssep) {
  using namespace GeographicLib;
  return dms ? DMS::Encode(azi, prec + 5, DMS::AZIMUTH, dmssep) :
    DMS::Encode(azi >= 180 ? azi - 360 : azi, prec + 5, DMS::NUMBER);
}

std::string DistanceStrings(real s12, real a12,
                            bool full, bool arcmode, int prec, bool dms) {
  using namespace GeographicLib;
  std::string s;
  if (full || !arcmode)
    s += Utility::str<real>(s12, prec);
  if (full)
    s += " ";
  if (full || arcmode)
    s += DMS::Encode(a12, prec + 5, dms ? DMS::NONE : DMS::NUMBER);
  return s;
}

real ReadDistance(const std::string& s, bool arcmode) {
  using namespace GeographicLib;
  return arcmode ? DMS::DecodeAngle(s) : Utility::num<real>(s);
}

int main(int argc, char* argv[]) {
  try {
    using namespace GeographicLib;
    bool linecalc = false, inverse = false, arcmode = false,
      dms = false, full = false, exact = false;
    real
      a = Constants::WGS84_a<real>(),
      f = Constants::WGS84_f<real>();
    real lat1, lon1, azi1, lat2, lon2, azi2, s12, m12, a12, M12, M21, S12;
    real azi2sense = 0;
    int prec = 3;
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';', dmssep = char(0);

    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-i") {
        inverse = true;
        linecalc = false;
      } else if (arg == "-a")
        arcmode = true;
      else if (arg == "-l") {
        inverse = false;
        linecalc = true;
        if (m + 3 >= argc) return usage(1, true);
        try {
          DMS::DecodeLatLon(std::string(argv[m + 1]), std::string(argv[m + 2]),
                            lat1, lon1);
          azi1 = DMS::DecodeAzimuth(std::string(argv[m + 3]));
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
      }
      else if (arg == "-d") {
        dms = true;
        dmssep = '\0';
      } else if (arg == "-:") {
        dms = true;
        dmssep = ':';
      } else if (arg == "-b")
        azi2sense = 180;
      else if (arg == "-f")
        full = true;
      else if (arg == "-p") {
        if (++m == argc) return usage(1, true);
        try {
          prec = Utility::num<int>(std::string(argv[m]));
        }
        catch (const std::exception&) {
          std::cerr << "Precision " << argv[m] << " is not a number\n";
          return 1;
        }
      } else if (arg == "-E")
        exact = true;
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

    const Geodesic geod(a, f);
    const GeodesicExact geode(a, f);
    GeodesicLine l;
    GeodesicLineExact le;
    if (linecalc) {
      if (exact)
        le = geode.Line(lat1, lon1, azi1);
      else
        l = geod.Line(lat1, lon1, azi1);
    }

    // Max precision = 10: 0.1 nm in distance, 10^-15 deg (= 0.11 nm),
    // 10^-11 sec (= 0.3 nm).
    prec = std::min(10 + Math::extradigits, std::max(0, prec));
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
        if (inverse) {
          std::string slat1, slon1, slat2, slon2;
          if (!(str >> slat1 >> slon1 >> slat2 >> slon2))
            throw GeographicErr("Incomplete input: " + s);
          std::string strc;
          if (str >> strc)
            throw GeographicErr("Extraneous input: " + strc);
          DMS::DecodeLatLon(slat1, slon1, lat1, lon1);
          DMS::DecodeLatLon(slat2, slon2, lat2, lon2);
          a12 = exact ?
            geode.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2,
                          m12, M12, M21, S12) :
            geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2,
                         m12, M12, M21, S12);
          if (full)
            *output << LatLonString(lat1, lon1, prec, dms, dmssep) << " ";
          *output << AzimuthString(azi1, prec, dms, dmssep) << " ";
          if (full)
            *output << LatLonString(lat2, lon2, prec, dms, dmssep) << " ";
          *output << AzimuthString(azi2 + azi2sense, prec, dms, dmssep) << " "
                  << DistanceStrings(s12, a12, full, arcmode, prec, dms);
          if (full)
            *output << " " << Utility::str<real>(m12, prec)
                    << " " << Utility::str<real>(M12, prec+7)
                    << " " << Utility::str<real>(M21, prec+7)
                    << " " << Utility::str<real>(S12, std::max(prec-7, 0));
          *output << eol;
        } else {
          if (linecalc) {
            std::string ss12;
            if (!(str >> ss12))
              throw GeographicErr("Incomplete input: " + s);
            std::string strc;
            if (str >> strc)
              throw GeographicErr("Extraneous input: " + strc);
            s12 = ReadDistance(ss12, arcmode);
            if (arcmode)
              exact ?
                le.ArcPosition(s12, lat2, lon2, azi2, a12, m12, M12, M21, S12) :
                l.ArcPosition(s12, lat2, lon2, azi2, a12, m12, M12, M21, S12);
            else
              a12 = exact ?
                le.Position(s12, lat2, lon2, azi2, m12, M12, M21, S12) :
                l.Position(s12, lat2, lon2, azi2, m12, M12, M21, S12);
          } else {
            std::string slat1, slon1, sazi1, ss12;
            if (!(str >> slat1 >> slon1 >> sazi1 >> ss12))
              throw GeographicErr("Incomplete input: " + s);
            std::string strc;
            if (str >> strc)
              throw GeographicErr("Extraneous input: " + strc);
            DMS::DecodeLatLon(slat1, slon1, lat1, lon1);
            azi1 = DMS::DecodeAzimuth(sazi1);
            s12 = ReadDistance(ss12, arcmode);
            if (arcmode)
              exact ?
                geode.ArcDirect(lat1, lon1, azi1, s12, lat2, lon2, azi2, a12,
                                m12, M12, M21, S12) :
                geod.ArcDirect(lat1, lon1, azi1, s12, lat2, lon2, azi2, a12,
                               m12, M12, M21, S12);
            else
              a12 = exact ?
                geode.Direct(lat1, lon1, azi1, s12, lat2, lon2, azi2,
                             m12, M12, M21, S12) :
                geod.Direct(lat1, lon1, azi1, s12, lat2, lon2, azi2,
                            m12, M12, M21, S12);
          }
          if (arcmode)
            std::swap(s12, a12);
          if (full)
            *output << LatLonString(lat1, lon1, prec, dms, dmssep) << " "
                    << AzimuthString(azi1, prec, dms, dmssep) << " ";
          *output << LatLonString(lat2, lon2, prec, dms, dmssep) << " "
                  << AzimuthString(azi2 + azi2sense, prec, dms, dmssep);
          if (full)
            *output << " "
                    << DistanceStrings(s12, a12, full, arcmode, prec, dms)
                    << " " << Utility::str<real>(m12, prec)
                    << " " << Utility::str<real>(M12, prec+7)
                    << " " << Utility::str<real>(M21, prec+7)
                    << " " << Utility::str<real>(S12, std::max(prec-7, 0));
          *output << eol;
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
