/**
 * \file GeoidEval.cpp
 * \brief Command line utility for evaluating geoid heights
 *
 * Copyright (c) Charles Karney (2009-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * See the <a href="GeoidEval.1.html">man page</a> for usage information.
 **********************************************************************/

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/GeoCoords.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions and potentially
// uninitialized local variables
#  pragma warning (disable: 4127 4701)
#endif

#include "GeoidEval.usage"

int main(int argc, const char* const argv[]) {
  try {
    using namespace GeographicLib;
    typedef Math::real real;
    Utility::set_digits();
    bool cacheall = false, cachearea = false, verbose = false, cubic = true;
    real caches, cachew, cachen, cachee;
    std::string dir;
    std::string geoid = Geoid::DefaultGeoidName();
    Geoid::convertflag heightmult = Geoid::NONE;
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';';
    bool northp = false, longfirst = false;
    int zonenum = UTMUPS::INVALID;

    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-a") {
        cacheall = true;
        cachearea = false;
      }
      else if (arg == "-c") {
        if (m + 4 >= argc) return usage(1, true);
        cacheall = false;
        cachearea = true;
        try {
          DMS::DecodeLatLon(std::string(argv[m + 1]), std::string(argv[m + 2]),
                            caches, cachew, longfirst);
          DMS::DecodeLatLon(std::string(argv[m + 3]), std::string(argv[m + 4]),
                            cachen, cachee, longfirst);
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding argument of -c: " << e.what() << "\n";
          return 1;
        }
        m += 4;
      } else if (arg == "--msltohae")
        heightmult = Geoid::GEOIDTOELLIPSOID;
      else if (arg == "--haetomsl")
        heightmult = Geoid::ELLIPSOIDTOGEOID;
      else if (arg == "-w")
        longfirst = !longfirst;
      else if (arg == "-z") {
        if (++m == argc) return usage(1, true);
        std::string zone = argv[m];
        try {
          UTMUPS::DecodeZone(zone, zonenum, northp);
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding zone: " << e.what() << "\n";
          return 1;
        }
        if (!(zonenum >= UTMUPS::MINZONE && zonenum <= UTMUPS::MAXZONE)) {
          std::cerr << "Illegal zone " << zone << "\n";
          return 1;
        }
      } else if (arg == "-n") {
        if (++m == argc) return usage(1, true);
        geoid = argv[m];
      } else if (arg == "-d") {
        if (++m == argc) return usage(1, true);
        dir = argv[m];
      } else if (arg == "-l")
        cubic = false;
      else if (arg == "-v")
        verbose = true;
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
      } else {
        int retval = usage(!(arg == "-h" || arg == "--help"), arg != "--help");
        if (arg == "-h")
          std::cout
            << "\nDefault geoid path = \""   << Geoid::DefaultGeoidPath()
            << "\"\nDefault geoid name = \"" << Geoid::DefaultGeoidName()
            << "\"\n";
        return retval;
      }
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

    int retval = 0;
    try {
      const Geoid g(geoid, dir, cubic);
      try {
        if (cacheall)
          g.CacheAll();
        else if (cachearea)
          g.CacheArea(caches, cachew, cachen, cachee);
      }
      catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\nProceeding without a cache\n";
      }
      if (verbose) {
        std::cerr << "Geoid file: "    << g.GeoidFile()     << "\n"
                  << "Description: "   << g.Description()   << "\n"
                  << "Interpolation: " << g.Interpolation() << "\n"
                  << "Date & Time: "   << g.DateTime()      << "\n"
                  << "Offset (m): "    << g.Offset()        << "\n"
                  << "Scale (m): "     << g.Scale()         << "\n"
                  << "Max error (m): " << g.MaxError()      << "\n"
                  << "RMS error (m): " << g.RMSError()      << "\n";
        if (g.Cache())
          std::cerr
            << "Caching:"
            << "\n SW Corner: " << g.CacheSouth() << " " << g.CacheWest()
            << "\n NE Corner: " << g.CacheNorth() << " " << g.CacheEast()
            << "\n";
      }

      GeoCoords p;
      std::string s, eol, suff;
      const char* spaces = " \t\n\v\f\r,"; // Include comma as space
      while (std::getline(*input, s)) {
        try {
          eol = "\n";
          if (!cdelim.empty()) {
            std::string::size_type m = s.find(cdelim);
            if (m != std::string::npos) {
              eol = " " + s.substr(m) + "\n";
              std::string::size_type m1 =
                m > 0 ? s.find_last_not_of(spaces, m - 1) : std::string::npos;
              s = s.substr(0, m1 != std::string::npos ? m1 + 1 : m);
            }
          }
          real height = 0;
          if (zonenum != UTMUPS::INVALID) {
            // Expect "easting northing" if heightmult == 0, or
            // "easting northing height" if heightmult != 0.
            std::string::size_type pa = 0, pb = 0;
            real easting = 0, northing = 0;
            for (int i = 0; i < (heightmult ? 3 : 2); ++i) {
              if (pb == std::string::npos)
                throw GeographicErr("Incomplete input: " + s);
              // Start of i'th token
              pa = s.find_first_not_of(spaces, pb);
              if (pa == std::string::npos)
                throw GeographicErr("Incomplete input: " + s);
              // End of i'th token
              pb = s.find_first_of(spaces, pa);
              (i == 2 ? height : (i == 0 ? easting : northing)) =
                Utility::val<real>(s.substr(pa, (pb == std::string::npos ?
                                                 pb : pb - pa)));
            }
            p.Reset(zonenum, northp, easting, northing);
            if (heightmult) {
              suff = pb == std::string::npos ? "" : s.substr(pb);
              s = s.substr(0, pa);
            }
          } else {
            if (heightmult) {
              // Treat last token as height
              // pb = last char of last token
              // pa = last char preceding white space
              // px = last char of 2nd last token
              std::string::size_type pb = s.find_last_not_of(spaces);
              std::string::size_type pa = s.find_last_of(spaces, pb);
              if (pa == std::string::npos || pb == std::string::npos)
                throw GeographicErr("Incomplete input: " + s);
              height = Utility::val<real>(s.substr(pa + 1, pb - pa));
              s = s.substr(0, pa + 1);
            }
            p.Reset(s, true, longfirst);
          }
          if (heightmult) {
            real h = g(p.Latitude(), p.Longitude());
            *output << s
                    << Utility::str(height + real(heightmult) * h, 4)
                    << suff << eol;
          } else {
            real h = g(p.Latitude(), p.Longitude());
            *output << Utility::str(h, 4) << eol;
          }
        }
        catch (const std::exception& e) {
          *output << "ERROR: " << e.what() << "\n";
          retval = 1;
        }
      }
    }
    catch (const std::exception& e) {
      std::cerr << "Error reading " << geoid << ": " << e.what() << "\n";
      retval = 1;
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
