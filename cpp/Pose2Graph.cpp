/**
 * @file    Pose2Graph.cpp
 * @brief   A factor graph for the 2D PoseSLAM problem
 * @authors Frank Dellaert, Viorela Ila
 */

#include "Pose2Graph.h"
#include "FactorGraph-inl.h"
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "NonlinearEquality.h"
#include "graph-inl.h"

using namespace std;
using namespace gtsam;

namespace gtsam {

	// explicit instantiation so all the code is there and we can link with it
	template class FactorGraph<NonlinearFactor<Pose2Config> > ;
	template class NonlinearFactorGraph<Pose2Config> ;
	template class NonlinearEquality<Pose2Config,Pose2Config::Key,Pose2> ;
	template class NonlinearOptimizer<Pose2Graph, Pose2Config>;

	void Pose2Graph::addConstraint(const Pose2Config::Key& key, const Pose2& pose) {
		push_back(sharedFactor(new NonlinearEquality<Pose2Config, Pose2Config::Key,
				Pose2> (key, pose)));
	}

	bool Pose2Graph::equals(const Pose2Graph& p, double tol) const {
		return false;
	}

} // namespace gtsam
