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
class VSLAMGraph : public gtsam::NonlinearFactorGraph<VSLAMConfig>, Testable<VSLAMGraph>{
private:
	int nFrames;
	typedef map <int, int> feat_ids_type;
	feat_ids_type feat_ids;

public:

  /** default constructor is empty graph */
  VSLAMGraph() {}

  /**
   * Constructor that loads measurements from file
   * @param path to the file
   */
  VSLAMGraph(const std::string& path);

  /**
   * print out graph
   */
  void print(const std::string& s = "") const {
    gtsam::NonlinearFactorGraph<VSLAMConfig>::print(s);
  }

  /**
   * equals
   */
  bool equals(const VSLAMGraph&, double tol=1e-9) const;

  // Getters
  int Get_nFrames(){return nFrames;};
  int Get_nFeat_ids(){return feat_ids.size();};
  feat_ids_type* Get_feat_ids_map(){return &feat_ids;};

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
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(nFrames);
		ar & BOOST_SERIALIZATION_NVP(feat_ids);
	}
};

} // namespace gtsam
