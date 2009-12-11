/**
 * @file    Pose2Graph.h
 * @brief   A factor graph for the 2D PoseSLAM problem
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#pragma once


#include "NonlinearFactorGraph.h"
#include "FactorGraph.h"
#include "Pose2Factor.h"
#include "Pose2Config.h"
#include "Testable.h"
#include "Ordering.h"

namespace gtsam{

/**
 * Non-linear factor graph for visual SLAM
 */
class Pose2Graph : public gtsam::NonlinearFactorGraph<Pose2Config>{

public:

  /** default constructor is empty graph */
  Pose2Graph() {}

  /**
   * print out graph
   */
  void print(const std::string& s = "") const;

  /**
   * equals
   */
  bool equals(const Pose2Graph& p, double tol=1e-9) const;


private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {}
};

} // namespace gtsam
