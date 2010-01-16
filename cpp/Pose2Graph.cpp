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
#include "LieConfig-inl.h"

using namespace std;
using namespace gtsam;

namespace gtsam {

  /** Explicit instantiation of templated methods and functions */
  template class LieConfig<Symbol<Pose2,'x'>,Pose2>;
  template size_t dim(const Pose2Config& c);
  template Pose2Config expmap(const Pose2Config& c, const Vector& delta);
  template Pose2Config expmap(const Pose2Config& c, const VectorConfig& delta);

	// explicit instantiation so all the code is there and we can link with it
	template class FactorGraph<NonlinearFactor<Pose2Config> > ;
	template class NonlinearFactorGraph<Pose2Config> ;
	template class NonlinearEquality<Pose2Config,Pose2Config::Key,Pose2> ;
	template class NonlinearOptimizer<Pose2Graph, Pose2Config>;

  /* ************************************************************************* */
  Pose2Config pose2Circle(size_t n, double R) {
    Pose2Config x;
    double theta = 0, dtheta = 2*M_PI/n;
    for(size_t i=0;i<n;i++, theta+=dtheta)
      x.insert(i, Pose2(cos(theta), sin(theta), M_PI_2 + theta));
    return x;
  }

  /* ************************************************************************* */
	void Pose2Graph::addConstraint(const Pose2Config::Key& key, const Pose2& pose) {
		push_back(sharedFactor(new NonlinearEquality<Pose2Config, Pose2Config::Key,
				Pose2> (key, pose)));
	}

  /* ************************************************************************* */
	bool Pose2Graph::equals(const Pose2Graph& p, double tol) const {
		return false;
	}

} // namespace gtsam
