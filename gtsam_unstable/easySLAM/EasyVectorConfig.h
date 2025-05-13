/**
 * @file   EasyVectorConfig.h
 * @author Alireza Fathi
 * @author Frank Dellaert
 */

#include <gtsam/FGConfig.h>
#include <gtsam/Pose3.h>

#include <map>
#include <string>
#include <vector>

#pragma once

// special FGConfig derived class that knows we are dealing with Pose3 objects
// should be more elgant in later version of gtsam
class EasyVectorConfig : public gtsam::FGConfig {
 public:
  gtsam::FGConfig operator+(const gtsam::FGConfig& delta) const;
  void operator+=(const gtsam::FGConfig& delta);
};
