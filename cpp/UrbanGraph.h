/**
 * @file    UrbanGraph.h
 * @brief   A factor graph for the Urban problem
 * @author Frank Dellaert
 * @author Viorela Ila
 */
#pragma once

#include <vector>
#include <map>
#include <set>
#include <fstream>

#include "NonlinearFactorGraph.h"
#include "FactorGraph-inl.h"
#include "UrbanMeasurement.h"
#include "Pose3Factor.h"
#include "UrbanConfig.h"
#include "Testable.h"

namespace gtsam{

/**
 * Non-linear factor graph for visual SLAM
 */
class UrbanGraph : public gtsam::NonlinearFactorGraph<UrbanConfig>{

public:

  /** default constructor is empty graph */
  UrbanGraph() {}

  /**
   * print out graph
   */
  void print(const std::string& s = "") const {
    gtsam::NonlinearFactorGraph<UrbanConfig>::print(s);
  }

  /**
   * equals
   */
  bool equals(const UrbanGraph& p, double tol=1e-9) const {
	  return gtsam::NonlinearFactorGraph<UrbanConfig>::equals(p, tol);
  }
  // TODO implement addMeasurenment, addOdometry and addOriginalConstraint (out of the class)
  /**
   *  Add a landmark constraint
   */
  void addMeasurement(double x, double y, double sigma, int p1, int p2) {};

  /**
   *  Add an odometry constraint
   */
  void addOdometry(double dx, double yaw, double sigmadx, double sigmayaw, int p  ) {};

  /**
   *  Add an initial constraint on the first pose
   */
  void addOriginConstraint(int p){};

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {}
};

} // namespace gtsam
