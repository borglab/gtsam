/**
 * \file GeodSolve.cpp
 * \brief Command line utility for geodesic calculations
 *
 * Copyright (c) Charles Karney (2009-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * See the <a href="GeodSolve.1.html">man page</a> for usage information.
 **********************************************************************/

#include <iostream>
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
  using namespace GeographicLib;
  return dms ? DMS::Encode(azi, prec + 5, DMS::AZIMUTH, dmssep) :
    DMS::Encode(azi, prec + 5, DMS::NUMBER);
}

std::string DistanceStrings(real s12, real a12,
                            bool full, bool arcmode, int prec, bool dms) {
  using namespace GeographicLib;
  std::string s;
  if (full || !arcmode)
    s += Utility::str(s12, prec);
  if (full)
    s += " ";
  if (full || arcmode)
    s += DMS::Encode(a12, prec + 5, dms ? DMS::NONE : DMS::NUMBER);
  return s;
}

real ReadDistance(const std::string& s, bool arcmode) {
  using namespace GeographicLib;
  return arcmode ? DMS::DecodeAngle(s) : Utility::val<real>(s);
}

int main(int argc, const char* const argv[]) {
  try {
    using namespace GeographicLib;
    enum { NONE = 0, LINE, DIRECT, INVERSE };
    Utility::set_digits();
    bool inverse = false, arcmode = false,
      dms = false, full = false, exact = false, unroll = false,
      longfirst = false, azi2back = false, fraction = false,
      arcmodeline = false;
    real
      a = Constants::WGS84_a(),
      f = Constants::WGS84_f();
    real lat1, lon1, azi1, lat2, lon2, azi2, s12, m12, a12, M12, M21, S12,
      mult = 1;
    int linecalc = NONE, prec = 3;
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';', dmssep = char(0);

    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-i") {
        inverse = true;
        linecalc = NONE;
      } else if (arg == "-a")
        arcmode = !arcmode;
      else if (arg == "-F")
        fraction = true;
      else if (arg == "-L" || arg == "-l") { // -l is DEPRECATED
        inverse = false;
        linecalc = LINE;
        if (m + 3 >= argc) return usage(1, true);
        try {
          DMS::DecodeLatLon(std::string(argv[m + 1]), std::string(argv[m + 2]),
                            lat1, lon1, longfirst);
          azi1 = DMS::DecodeAzimuth(std::string(argv[m + 3]));
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding arguments of -L: " << e.what() << "\n";
          return 1;
        }
        m += 3;
      } else if (arg == "-D") {
        inverse = false;
        linecalc = DIRECT;
        if (m + 4 >= argc) return usage(1, true);
        try {
          DMS::DecodeLatLon(std::string(argv[m + 1]), std::string(argv[m + 2]),
                            lat1, lon1, longfirst);
          azi1 = DMS::DecodeAzimuth(std::string(argv[m + 3]));
          s12 = ReadDistance(std::string(argv[m + 4]), arcmode);
          arcmodeline = arcmode;
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding arguments of -D: " << e.what() << "\n";
          return 1;
        }
        m += 4;
      } else if (arg == "-I") {
        inverse = false;
        linecalc = INVERSE;
        if (m + 4 >= argc) return usage(1, true);
        try {
          DMS::DecodeLatLon(std::string(argv[m + 1]), std::string(argv[m + 2]),
                            lat1, lon1, longfirst);
          DMS::DecodeLatLon(std::string(argv[m + 3]), std::string(argv[m + 4]),
                            lat2, lon2, longfirst);
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding arguments of -I: " << e.what() << "\n";
          return 1;
        }
        m += 4;
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
      } else if (arg == "-u")
        unroll = true;
      else if (arg == "-d") {
        dms = true;
        dmssep = '\0';
      } else if (arg == "-:") {
        dms = true;
        dmssep = ':';
      } else if (arg == "-w")
        longfirst = !longfirst;
      else if (arg == "-b")
        azi2back = true;
      else if (arg == "-f")
        full = true;
      else if (arg == "-p") {
        if (++m == argc) return usage(1, true);
        try {
          prec = Utility::val<int>(std::string(argv[m]));
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

    // GeodesicExact mask values are the same as Geodesic
    unsigned outmask = Geodesic::LATITUDE | Geodesic::LONGITUDE |
      Geodesic::AZIMUTH;        // basic output quantities
    outmask |= inverse ? Geodesic::DISTANCE : // distance-related flags
      (arcmode ? Geodesic::NONE : Geodesic::DISTANCE_IN);
    // longitude unrolling
    outmask |= unroll ? Geodesic::LONG_UNROLL : Geodesic::NONE;
    // full output -- don't use Geodesic::ALL since this includes DISTANCE_IN
    outmask |= full ? (Geodesic::DISTANCE | Geodesic::REDUCEDLENGTH |
                       Geodesic::GEODESICSCALE | Geodesic::AREA) :
      Geodesic::NONE;

    const Geodesic      geods(a, f);
    const GeodesicExact geode(a, f);
    GeodesicLine      ls;
    GeodesicLineExact le;
    if (linecalc) {
      if (linecalc == LINE) fraction = false;
      if (exact) {
        le = linecalc == DIRECT ?
          geode.GenDirectLine(lat1, lon1, azi1, arcmodeline, s12, outmask) :
          linecalc == INVERSE ?
          geode.InverseLine(lat1, lon1, lat2, lon2, outmask) :
          // linecalc == LINE
          geode.Line(lat1, lon1, azi1, outmask);
        mult = fraction ? le.GenDistance(arcmode) : 1;
        if (linecalc == INVERSE) azi1 = le.Azimuth();
      } else {
        ls = linecalc == DIRECT ?
          geods.GenDirectLine(lat1, lon1, azi1, arcmodeline, s12, outmask) :
          linecalc == INVERSE ?
          geods.InverseLine(lat1, lon1, lat2, lon2, outmask) :
          // linecalc == LINE
          geods.Line(lat1, lon1, azi1, outmask);
        mult = fraction ? ls.GenDistance(arcmode) : 1;
        if (linecalc == INVERSE) azi1 = ls.Azimuth();
      }
    }

    // Max precision = 10: 0.1 nm in distance, 10^-15 deg (= 0.11 nm),
    // 10^-11 sec (= 0.3 nm).
    prec = std::min(10 + Math::extra_digits(), std::max(0, prec));
    std::string s, eol, slat1, slon1, slat2, slon2, sazi1, ss12, strc;
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
        if (inverse) {
          if (!(str >> slat1 >> slon1 >> slat2 >> slon2))
            throw GeographicErr("Incomplete input: " + s);
          if (str >> strc)
            throw GeographicErr("Extraneous input: " + strc);
          DMS::DecodeLatLon(slat1, slon1, lat1, lon1, longfirst);
          DMS::DecodeLatLon(slat2, slon2, lat2, lon2, longfirst);
          a12 = exact ?
            geode.GenInverse(lat1, lon1, lat2, lon2, outmask,
                             s12, azi1, azi2, m12, M12, M21, S12) :
            geods.GenInverse(lat1, lon1, lat2, lon2, outmask,
                             s12, azi1, azi2, m12, M12, M21, S12);
          if (full) {
            if (unroll) {
              real e;
              lon2 = lon1 + Math::AngDiff(lon1, lon2, e);
              lon2 += e;
            } else {
              lon1 = Math::AngNormalize(lon1);
              lon2 = Math::AngNormalize(lon2);
            }
            *output << LatLonString(lat1, lon1, prec, dms, dmssep, longfirst)
                    << " ";
          }
          *output << AzimuthString(azi1, prec, dms, dmssep) << " ";
          if (full)
            *output << LatLonString(lat2, lon2, prec, dms, dmssep, longfirst)
                    << " ";
          if (azi2back)
            azi2 += azi2 >= 0 ? -180 : 180;
          *output << AzimuthString(azi2, prec, dms, dmssep) << " "
                  << DistanceStrings(s12, a12, full, arcmode, prec, dms);
          if (full)
            *output << " " << Utility::str(m12, prec)
                    << " " << Utility::str(M12, prec+7)
                    << " " << Utility::str(M21, prec+7)
                    << " " << Utility::str(S12, std::max(prec-7, 0));
          *output << eol;
        } else {
          if (linecalc) {
            if (!(str >> ss12))
              throw GeographicErr("Incomplete input: " + s);
            if (str >> strc)
              throw GeographicErr("Extraneous input: " + strc);
            // In fraction mode input is read as a distance
            s12 = ReadDistance(ss12, !fraction && arcmode) * mult;
            a12 = exact ?
              le.GenPosition(arcmode, s12, outmask,
                             lat2, lon2, azi2, s12, m12, M12, M21, S12) :
              ls.GenPosition(arcmode, s12, outmask,
                             lat2, lon2, azi2, s12, m12, M12, M21, S12);
          } else {
            if (!(str >> slat1 >> slon1 >> sazi1 >> ss12))
              throw GeographicErr("Incomplete input: " + s);
            if (str >> strc)
              throw GeographicErr("Extraneous input: " + strc);
            DMS::DecodeLatLon(slat1, slon1, lat1, lon1, longfirst);
            azi1 = DMS::DecodeAzimuth(sazi1);
            s12 = ReadDistance(ss12, arcmode);
            a12 = exact ?
              geode.GenDirect(lat1, lon1, azi1, arcmode, s12, outmask,
                              lat2, lon2, azi2, s12, m12, M12, M21, S12) :
              geods.GenDirect(lat1, lon1, azi1, arcmode, s12, outmask,
                              lat2, lon2, azi2, s12, m12, M12, M21, S12);
          }
          if (full)
            *output
              << LatLonString(lat1, unroll ? lon1 : Math::AngNormalize(lon1),
                              prec, dms, dmssep, longfirst)
              << " " << AzimuthString(azi1, prec, dms, dmssep) << " ";
          if (azi2back)
            azi2 += azi2 >= 0 ? -180 : 180;
          *output << LatLonString(lat2, lon2, prec, dms, dmssep, longfirst)
                  << " " << AzimuthString(azi2, prec, dms, dmssep);
          if (full)
            *output << " "
                    << DistanceStrings(s12, a12, full, arcmode, prec, dms)
                    << " " << Utility::str(m12, prec)
                    << " " << Utility::str(M12, prec+7)
                    << " " << Utility::str(M21, prec+7)
                    << " " << Utility::str(S12, std::max(prec-7, 0));
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
