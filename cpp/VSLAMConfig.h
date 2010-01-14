/**
 * @file   VSLAMConfig.h
 * @brief  Config for VSLAM
 * @author Alireza Fathi
 * @author Carlos Nieto
 */

#include "Pose3.h"
#include "Point3.h"
#include "TupleConfig.h"

#pragma once

namespace gtsam{

  typedef Symbol<Pose3,'x'> VSLAMPoseKey;
  typedef Symbol<Point3,'l'> VSLAMPointKey;
  typedef PairConfig<VSLAMPoseKey, Pose3, VSLAMPointKey, Point3> VSLAMConfig;

}
