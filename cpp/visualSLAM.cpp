/*
 * visualSLAM.cpp
 *
 *  Created on: Jan 14, 2010
 *      Author: richard
 */

#include "visualSLAM.h"
#include "TupleConfig-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "NonlinearFactorGraph-inl.h"

namespace gtsam {

  INSTANTIATE_PAIR_CONFIG(visualSLAM::PoseKey, Pose3, visualSLAM::PointKey, Point3)
  INSTANTIATE_NONLINEAR_FACTOR_GRAPH(visualSLAM::Config)
  INSTANTIATE_NONLINEAR_OPTIMIZER(visualSLAM::Graph, visualSLAM::Config)

  namespace visualSLAM {

    /* ************************************************************************* */
    void ProjectionFactor::print(const std::string& s) const {
      Base::print(s);
      z_.print(s + ".z");
    }

    /* ************************************************************************* */
    bool ProjectionFactor::equals(const ProjectionFactor& p, double tol) const {
      return Base::equals(p, tol) && z_.equals(p.z_, tol)
                && K_->equals(*p.K_, tol);
    }

    //  /* ************************************************************************* */
    //  bool compareLandmark(const std::string& key,
    //            const VSLAMConfig& feasible,
    //            const VSLAMConfig& input) {
    //    int j = atoi(key.substr(1, key.size() - 1).c_str());
    //    return feasible[VSLAMPointKey(j)].equals(input[VSLAMPointKey(j)]);
    //  }
    //
    //  /* ************************************************************************* */
    //  void VSLAMGraph::addLandmarkConstraint(int j, const Point3& p) {
    //    typedef NonlinearEquality<VSLAMConfig,VSLAMPointKey,Point3> NLE;
    //    boost::shared_ptr<NLE> factor(new NLE(j, p));
    //    push_back(factor);
    //  }
    //
    //  /* ************************************************************************* */
    //  bool compareCamera(const std::string& key,
    //            const VSLAMConfig& feasible,
    //            const VSLAMConfig& input) {
    //    int j = atoi(key.substr(1, key.size() - 1).c_str());
    //    return feasible[VSLAMPoseKey(j)].equals(input[VSLAMPoseKey(j)]);
    //  }
    //
    //  /* ************************************************************************* */
    //  void VSLAMGraph::addCameraConstraint(int j, const Pose3& p) {
    //    typedef NonlinearEquality<VSLAMConfig,VSLAMPoseKey,Pose3> NLE;
    //    boost::shared_ptr<NLE> factor(new NLE(j,p));
    //    push_back(factor);
    //  }
  }
}
