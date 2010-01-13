/**
 * @file    Pose2Config.cpp
 * @brief   Configuration of 2D poses
 * @author  Frank Dellaert
 */

#include "Pose2Config.h"
#include "LieConfig-inl.h"

using namespace std;

namespace gtsam {

  /** Explicit instantiation of templated methods and functions */
  template class LieConfig<Symbol<Pose2,'x'>,Pose2>;
  template size_t dim(const Pose2Config& c);
  template Pose2Config expmap(const Pose2Config& c, const Vector& delta);
  template Pose2Config expmap(const Pose2Config& c, const VectorConfig& delta);

	/* ************************************************************************* */
	Pose2Config pose2Circle(size_t n, double R) {
		Pose2Config x;
		double theta = 0, dtheta = 2*M_PI/n;
		for(size_t i=0;i<n;i++, theta+=dtheta)
			x.insert(i, Pose2(cos(theta), sin(theta), M_PI_2 + theta));
		return x;
	}

	/* ************************************************************************* */
} // namespace
