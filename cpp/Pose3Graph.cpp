/**
 * @file    Pose3Graph.cpp
 * @brief   A factor graph for the 2D PoseSLAM problem
 * @authors Frank Dellaert, Viorela Ila
 */
//#include "NonlinearOptimizer-inl.h"
#include "FactorGraph-inl.h"
#include "NonlinearFactorGraph-inl.h"
#include "Pose3Graph.h"

namespace gtsam {

// explicit instantiation so all the code is there and we can link with it
template class FactorGraph<NonlinearFactor<gtsam::Pose3Config> > ;
template class NonlinearFactorGraph<Pose3Config> ;
//template class NonlinearOptimizer<Pose3Graph, Pose3Config> ;

bool Pose3Graph::equals(const Pose3Graph& p, double tol) const {
	return false;
}

} // namespace gtsam
