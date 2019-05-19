/**
 * \file Gravity.cpp
 * \brief Command line utility for evaluating gravity fields
 *
 * Copyright (c) Charles Karney (2011-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * See the <a href="Gravity.1.html">man page</a> for usage information.
 **********************************************************************/

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <GeographicLib/GravityModel.hpp>
#include <GeographicLib/GravityCircle.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions and potentially
// uninitialized local variables
#  pragma warning (disable: 4127 4701)
#endif

#include "Gravity.usage"

int main(int argc, const char* const argv[]) {
  try {
    using namespace GeographicLib;
    typedef Math::real real;
    Utility::set_digits();
    bool verbose = false, longfirst = false;
    std::string dir;
    std::string model = GravityModel::DefaultGravityName();
    std::string istring, ifile, ofile, cdelim;
    char lsep = ';';
    real lat = 0, h = 0;
    bool circle = false;
    int prec = -1;
    enum {
      GRAVITY = 0,
      DISTURBANCE = 1,
      ANOMALY = 2,
      UNDULATION = 3,
    };
    unsigned mode = GRAVITY;
    for (int m = 1; m < argc; ++m) {
      std::string arg(argv[m]);
      if (arg == "-n") {
        if (++m == argc) return usage(1, true);
        model = argv[m];
      } else if (arg == "-d") {
        if (++m == argc) return usage(1, true);
        dir = argv[m];
      } else if (arg == "-G")
        mode = GRAVITY;
      else if (arg == "-D")
        mode = DISTURBANCE;
      else if (arg == "-A")
        mode = ANOMALY;
      else if (arg == "-H")
        mode = UNDULATION;
      else if (arg == "-c") {
        if (m + 2 >= argc) return usage(1, true);
        try {
          using std::abs;
          DMS::flag ind;
          lat = DMS::Decode(std::string(argv[++m]), ind);
          if (ind == DMS::LONGITUDE)
            throw GeographicErr("Bad hemisphere letter on latitude");
          if (!(abs(lat) <= 90))
            throw GeographicErr("Latitude not in [-90d, 90d]");
          h = Utility::val<real>(std::string(argv[++m]));
          circle = true;
        }
        catch (const std::exception& e) {
          std::cerr << "Error decoding argument of " << arg << ": "
                    << e.what() << "\n";
          return 1;
        }
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
      } else if (arg == "-v")
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
          std::cout<< "\nDefault gravity path = \""
                   << GravityModel::DefaultGravityPath()
                   << "\"\nDefault gravity name = \""
                   << GravityModel::DefaultGravityName()
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

    switch (mode) {
    case GRAVITY:
      prec = std::min(16 + Math::extra_digits(), prec < 0 ? 5 : prec);
      break;
    case DISTURBANCE:
    case ANOMALY:
      prec = std::min(14 + Math::extra_digits(), prec < 0 ? 3 : prec);
      break;
    case UNDULATION:
    default:
      prec = std::min(12 + Math::extra_digits(), prec < 0 ? 4 : prec);
      break;
    }
    int retval = 0;
    try {
      const GravityModel g(model, dir);
      if (circle) {
        if (!Math::isfinite(h))
          throw GeographicErr("Bad height");
        else if (mode == UNDULATION && h != 0)
          throw GeographicErr("Height should be zero for geoid undulations");
      }
      if (verbose) {
        std::cerr << "Gravity file: " << g.GravityFile()      << "\n"
                  << "Name: "         << g.GravityModelName() << "\n"
                  << "Description: "  << g.Description()      << "\n"
                  << "Date & Time: "  << g.DateTime()         << "\n";
      }
      unsigned mask = (mode == GRAVITY ? GravityModel::GRAVITY :
                       (mode == DISTURBANCE ? GravityModel::DISTURBANCE :
                        (mode == ANOMALY ? GravityModel::SPHERICAL_ANOMALY :
                         GravityModel::GEOID_HEIGHT))); // mode == UNDULATION
      const GravityCircle c(circle ? g.Circle(lat, h, mask) : GravityCircle());
      std::string s, eol, stra, strb;
      std::istringstream str;
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
          real lon;
          if (circle) {
            if (!(str >> strb))
              throw GeographicErr("Incomplete input: " + s);
            DMS::flag ind;
            lon = DMS::Decode(strb, ind);
            if (ind == DMS::LATITUDE)
              throw GeographicErr("Bad hemisphere letter on " + strb);
          } else {
            if (!(str >> stra >> strb))
              throw GeographicErr("Incomplete input: " + s);
            DMS::DecodeLatLon(stra, strb, lat, lon, longfirst);
            h = 0;
            if (!(str >> h))    // h is optional
              str.clear();
            if (mode == UNDULATION && h != 0)
                throw GeographicErr("Height must be zero for geoid heights");
          }
          if (str >> stra)
            throw GeographicErr("Extra junk in input: " + s);
          switch (mode) {
          case GRAVITY:
            {
              real gx, gy, gz;
              if (circle) {
                c.Gravity(lon, gx, gy, gz);
              } else {
                g.Gravity(lat, lon, h, gx, gy, gz);
              }
              *output << Utility::str(gx, prec) << " "
                      << Utility::str(gy, prec) << " "
                      << Utility::str(gz, prec) << eol;
            }
            break;
          case DISTURBANCE:
            {
              real deltax, deltay, deltaz;
              if (circle) {
                c.Disturbance(lon, deltax, deltay, deltaz);
              } else {
                g.Disturbance(lat, lon, h, deltax, deltay, deltaz);
              }
              // Convert to mGals
              *output << Utility::str(deltax * 100000, prec) << " "
                      << Utility::str(deltay * 100000, prec) << " "
                      << Utility::str(deltaz * 100000, prec)
                      << eol;
            }
            break;
          case ANOMALY:
            {
              real Dg01, xi, eta;
              if (circle)
                c.SphericalAnomaly(lon, Dg01, xi, eta);
              else
                g.SphericalAnomaly(lat, lon, h, Dg01, xi, eta);
              Dg01 *= 100000;      // Convert to mGals
              xi *= 3600;       // Convert to arcsecs
              eta *= 3600;
              *output << Utility::str(Dg01, prec) << " "
                      << Utility::str(xi, prec) << " "
                      << Utility::str(eta, prec) << eol;
            }
            break;
          case UNDULATION:
          default:
            {
              real N = circle ? c.GeoidHeight(lon) : g.GeoidHeight(lat, lon);
              *output << Utility::str(N, prec) << eol;
            }
            break;
          }
        }
        catch (const std::exception& e) {
          *output << "ERROR: " << e.what() << "\n";
          retval = 1;
        }
      }
    }
    catch (const std::exception& e) {
      std::cerr << "Error reading " << model << ": " << e.what() << "\n";
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
