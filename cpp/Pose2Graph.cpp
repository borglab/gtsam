/**
 * @file    Pose2Graph.cpp
 * @brief   A factor graph for the 2D PoseSLAM problem
 * @authors Frank Dellaert, Viorela Ila
 */
#include "FactorGraph-inl.h"
#include "NonlinearFactorGraph-inl.h"
#include "graph-inl.h"
#include "Pose2Graph.h"

using namespace std;
using namespace gtsam;

namespace gtsam {

// explicit instantiation so all the code is there and we can link with it
template class FactorGraph<NonlinearFactor<gtsam::Pose2Config> > ;
template class NonlinearFactorGraph<Pose2Config> ;
//template class NonlinearOptimizer<Pose2Graph, Pose2Config> ;

/* ************************************************************************* */

bool Pose2Graph::equals(const Pose2Graph& p, double tol) const {
	return false;
}

/* ************************************************************************* */

} // namespace gtsam
