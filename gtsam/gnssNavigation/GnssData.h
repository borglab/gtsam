/**
 * @file   GnssData.h
 * @brief  Tools required to read/write GNSS data
 * @author Ryan Watson
 */


#pragma once

#include <gtsam/config.h>
#include <gtsam/dllexport.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/gnssNavigation/gnssStateVec.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/robustModels/PseudorangeSwitchFactor.h>

#include "boost/foreach.hpp"

#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace boost;

#define foreach BOOST_FOREACH

namespace gtsam {

/// Read GNSS data in the rnxToGtsam.cpp format
/// Data = { Week, Sow, Epoch, SVN, SatXYZ, Rho, P.C., L.C., Break_Flag}
typedef boost::tuple<double, int, int, Point3, double, double, double, int> rnxData;
vector<rnxData> readGNSS(const std::string& fileLoc);

/// Write GNSS states to text file
void writeStates(Values &results, string outputFile);

/// Write Pos. solution in ENU co-ordinate frame
void writeNavFrame(Values &results, Point3 &nom, string outputFile);

/// Write Pos. solution in ECEF co-ordinate frame
void writeEarthFrame(Values &results, Point3 &nom, string outputFile);

/// Write switch states to text file
void writeSwitches( Values &results, string outputFile, vector<string> switchIndex);

/// Write switch states to text file
void writeAmbiguity( Values &results, string outputFile, vector<string> satIndex);

}
