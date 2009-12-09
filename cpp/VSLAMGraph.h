/**
 * @file    VSLAMGraph.h
 * @brief   A factor graph for the VSLAM problem
 * @author  Alireza Fathi
 * @author  Carlos Nieto
 */

#pragma once

#include <vector>
#include <map>
#include <set>
#include <fstream>

#include "NonlinearFactorGraph.h"
#include "FactorGraph-inl.h"
#include "VSLAMFactor.h"
#include "VSLAMConfig.h"
#include "Testable.h"

namespace gtsam{

/**
 * Non-linear factor graph for visual SLAM
 */
class VSLAMGraph : public gtsam::NonlinearFactorGraph<VSLAMConfig>{

public:

  /** default constructor is empty graph */
  VSLAMGraph() {}

  /**
   * print out graph
   */
  void print(const std::string& s = "") const {
    gtsam::NonlinearFactorGraph<VSLAMConfig>::print(s);
  }

  /**
   * equals
   */
  bool equals(const VSLAMGraph& p, double tol=1e-9) const {
  	return gtsam::NonlinearFactorGraph<VSLAMConfig>::equals(p, tol);
  }

  /**
   *  Add a constraint on a landmark (for now, *must* be satisfied in any Config)
   *  @param j index of landmark
   *  @param p to which point to constrain it to
   */
  void addLandmarkConstraint(int j, const Point3& p = Point3());

  /**
   *  Add a constraint on a camera (for now, *must* be satisfied in any Config)
   *  @param j index of camera
   *  @param p to which pose to constrain it to
   */
  void addCameraConstraint(int j, const Pose3& p = Pose3());

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {}
};

} // namespace gtsam
