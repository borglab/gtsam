/**
 * @file   EasyVectorConfig.cpp
 * @brief  The Config
 * @author Alireza Fathi
 * @author Frank Dellaert
 */

#include "EasyVectorConfig.h"

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

// trick from some reading group
#define FOREACH_PAIR(KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY, VAL), COL)

/* ************************************************************************* */
FGConfig EasyVectorConfig::operator+(const FGConfig &delta) const {
  EasyVectorConfig result;
  string j;
  Vector v;
  FOREACH_PAIR(j, v, values) {
    Vector d = delta[j];
    Pose3 basePose(v);
    Pose3 newPose;
    newPose = basePose.exmap(d);
    result.insert(j, newPose.vector());
  }
  return result;
}

/* ************************************************************************* */
void EasyVectorConfig::operator+=(const FGConfig &delta) {
  for (iterator it = values.begin(); it != values.end(); it++) {
    string j = it->first;
    Vector &v = it->second;  // reference !
    Vector d = delta[j];
    Pose3 basePose(v);
    Pose3 newPose;
    newPose = basePose.exmap(d);
    v = newPose.vector();  // cahnges the reference in the config in-place !!!
  }
}

/* ************************************************************************* */
/* ************************************************************************* */
