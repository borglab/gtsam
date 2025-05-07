/**
 * \file IntersectTool.cpp
 * \brief Command line utility for geodesic intersections
 *
 * Copyright (c) Charles Karney (2023) <karney@alum.mit.edu> and licensed under
 * the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * See the <a href="IntersectTool.1.html">man page</a> for usage information.
 **********************************************************************/

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/Intersect.hpp>

#include "IntersectTool.usage"
using namespace GeographicLib;
typedef Math::real real;

int main(int argc, const char* const argv[]) {
  try {
    enum { CLOSE = 0, OFFSET, NEXT, SEGMENT };
    Utility::set_digits();
    real
      a = Constants::WGS84_a(),
      f = Constants::WGS84_f(),
      maxdist = -1;
    bool exact = false, check = false, longfirst = false;
    int prec = 3, mode = CLOSE;
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';';

    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-e") {
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
      } else if (arg == "-E")
        exact = true;
      else if (arg == "-p") {
        if (++m == argc) return usage(1, true);
        try {
          prec = Utility::val<int>(std::string(argv[m]));
        }
        catch (const std::exception&) {
          std::cerr << "Precision " << argv[m] << " is not a number\n";
          return 1;
        }
      } else if (arg == "-R") {
        if (++m == argc) return usage(1, true);
        try {
          maxdist = Utility::val<real>(std::string(argv[m]));
        }
        catch (const std::exception&) {
          std::cerr << "Maxdist " << argv[m] << " is not a number\n";
          return 1;
        }
        if (!(maxdist >= 0)) {
          std::cerr << "Maxdist must be nonnegative\n";
          return 1;
        }
      } else if (arg == "-c")
        mode = CLOSE;
      else if (arg == "-o")
        mode = OFFSET;
      else if (arg == "-n")
        mode = NEXT;
      else if (arg == "-i")
        mode = SEGMENT;
      else if (arg == "-C")
        check = true;
      else if (arg == "-w")
        longfirst = true;
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

    Geodesic geod(a, f, exact);
    Intersect intersect(geod);
    real latX1, lonX1, aziX, latY1, lonY1, aziY, latX2, lonX2, latY2, lonY2,
      x0 = 0, y0 = 0, x, y;
    std::string inp[8], s, sc, eol;
    std::istringstream str;
    int retval = 0,
      ninp = mode == CLOSE ? 6 : (mode == NEXT ? 4 :
                                  8); // mode == OFFSET || mode == SEGMENT
    GeodesicLine lineX, lineY;
    unsigned caps = Intersect::LineCaps;
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
        for (int i = 0; i < ninp; ++i) {
          if (!(str >> inp[i]))
          throw GeographicErr("Incomplete input: " + s);
        }
        if (str >> sc)
          throw GeographicErr("Extraneous input: " + sc);
        if (mode == CLOSE || mode == OFFSET) {
          DMS::DecodeLatLon(inp[0], inp[1], latX1, lonX1, longfirst);
          aziX = DMS::DecodeAzimuth(inp[2]);
          DMS::DecodeLatLon(inp[3], inp[4], latY1, lonY1, longfirst);
          aziY = DMS::DecodeAzimuth(inp[5]);
          if (mode == OFFSET) {
            x0 = Utility::val<real>(inp[6]);
            y0 = Utility::val<real>(inp[7]);
          } else
            x0 = y0 = 0;
          lineX = geod.Line(latX1, lonX1, aziX, caps);
          lineY = geod.Line(latY1, lonY1, aziY, caps);
        } else if (mode == NEXT) {
          DMS::DecodeLatLon(inp[0], inp[1], latX1, lonX1, longfirst);
          aziX = DMS::DecodeAzimuth(inp[2]);
          aziY = DMS::DecodeAzimuth(inp[3]);
          lineX = geod.Line(latX1, lonX1, aziX, caps);
          lineY = geod.Line(latX1, lonX1, aziY, caps);
        } else {                // mode == SEGMENT
          DMS::DecodeLatLon(inp[0], inp[1], latX1, lonX1, longfirst);
          DMS::DecodeLatLon(inp[2], inp[3], latX2, lonX2, longfirst);
          DMS::DecodeLatLon(inp[4], inp[5], latY1, lonY1, longfirst);
          DMS::DecodeLatLon(inp[6], inp[7], latY2, lonY2, longfirst);
          lineX = geod.InverseLine(latX1, lonX1, latX2, lonX2,
                                   Intersect::LineCaps);
          lineY = geod.InverseLine(latY1, lonY1, latY2, lonY2,
                                   Intersect::LineCaps);
          x0 = lineX.Distance()/2;
          y0 = lineY.Distance()/2;
        }
        std::pair<real, real> p0(x0, y0);
        if (maxdist < 0) {
          int segmode = 0, c;
          auto p = mode == CLOSE || mode == OFFSET ?
            intersect.Closest(lineX, lineY, p0, &c) :
            mode == NEXT ? intersect.Next(lineX, lineY, &c) :
            intersect.Segment(lineX, lineY, segmode, &c);
          x = p.first; y = p.second;
          *output << Utility::str(x, prec) << " "
                  << Utility::str(y, prec) << " " << c;
          if (mode == SEGMENT)
            *output << " " << segmode;
          *output << eol;
          if (check) {
            lineX.Position(x, latX2, lonX2);
            lineY.Position(y, latY2, lonY2);
            real sXY;
            geod.Inverse(latX2, lonX2, latY2, lonY2, sXY);
            std::cerr << Utility::str(longfirst ? lonX2 : latX2, prec+5) << " "
                      << Utility::str(longfirst ? latX2 : lonX2, prec+5) << " "
                      << Utility::str(longfirst ? lonY2 : latY2, prec+5) << " "
                      << Utility::str(longfirst ? latY2 : lonY2, prec+5) << " "
                      << Utility::str(sXY, prec) << eol;
          }
        } else {
          std::vector<int> c;
          auto v = intersect.All(lineX, lineY, maxdist, c, p0);
          unsigned n = unsigned(v.size());
          for (unsigned i = 0; i < n; ++i) {
            x = v[i].first; y = v[i].second;
            *output << Utility::str(x, prec) << " " << Utility::str(y, prec)
                    << " " << c[i] << " "
                    << Utility::str(Intersect::Dist(v[i], p0), prec)
                    << eol;
            if (check) {
              lineX.Position(x, latX2, lonX2);
              lineY.Position(y, latY2, lonY2);
              real sXY;
              geod.Inverse(latX2, lonX2, latY2, lonY2, sXY);
              std::cerr << Utility::str(longfirst ? lonX2 : latX2, prec+5) << " "
                        << Utility::str(longfirst ? latX2 : lonX2, prec+5) << " "
                        << Utility::str(longfirst ? lonY2 : latY2, prec+5) << " "
                        << Utility::str(longfirst ? latY2 : lonY2, prec+5) << " "
                        << Utility::str(sXY, prec) << eol;
            }
          }
          *output << "nan nan 0 nan" << eol;
          if (check)
            std::cerr << "nan nan nan nan nan" << eol;
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
