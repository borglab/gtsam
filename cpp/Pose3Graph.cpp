/**
 * @file    Pose3Graph.cpp
 * @brief   A factor graph for the 2D PoseSLAM problem
 * @authors Frank Dellaert, Viorela Ila
 */

#include "Pose3Graph.h"
#include "FactorGraph-inl.h"
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "NonlinearEquality.h"
#include "graph-inl.h"

namespace gtsam {

	// explicit instantiation so all the code is there and we can link with it
	template class FactorGraph<NonlinearFactor<gtsam::Pose3Config> > ;
	template class NonlinearFactorGraph<Pose3Config> ;
	template class NonlinearEquality<Pose3Config, Pose3Config::Key, Pose3> ;
	template class NonlinearOptimizer<Pose3Graph, Pose3Config> ;

	bool Pose3Graph::equals(const Pose3Graph& p, double tol) const {
		return false;
	}

	/**
	 *  Add an equality constraint on a pose
	 *  @param key of pose
	 *  @param pose which pose to constrain it to
	 */
	void Pose3Graph::addConstraint(const Pose3Config::Key& key, const Pose3& pose) {
		push_back(sharedFactor(new NonlinearEquality<Pose3Config, Pose3Config::Key,
				Pose3> (key, pose)));
	}

} // namespace gtsam
