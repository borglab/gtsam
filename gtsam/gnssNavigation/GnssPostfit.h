/**
 *  @file   GNSSPostfit.h
 *  @author Ryan
 *  @brief  Header file for GNSS postfit analysis
 **/

#pragma once
#include <numeric> 

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point4.h>
#include <gtsam/gnssNavigation/GnssData.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/gnssNavigation/gnssStateVec.h>

#ifndef FOREACH_HPP
  #define FOREACH_HPP
  #include <boost/foreach.hpp>
  #define foreach BOOST_FOREACH
#endif

#include "boost/foreach.hpp"

using namespace std;
using namespace boost;

#define foreach BOOST_FOREACH

namespace gtsam {

/// Function to calculate gnss postfit residuals
vector<double> getResiduals( Point3 &nomXYZ, Values &results, vector<rnxData> data);

/// write residuals to text file
void writeResiduals( vector<double> postfitResiduals, string outputFile, vector<string> switchIndex );

// iterate over residual vector to mark outliers.
vector<int> markResiduals( vector<double> postfitResdiuals, double threshold );

}
