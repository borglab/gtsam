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
  template class LieConfig<Pose2>;
  template size_t dim(const LieConfig<Pose2>& c);
  template LieConfig<Pose2> expmap(const LieConfig<Pose2>& c, const Vector& delta);
  template LieConfig<Pose2> expmap(const LieConfig<Pose2>& c, const VectorConfig& delta);

	/* ************************************************************************* */
	// TODO: local version, should probably defined in LieConfig
	static string symbol(char c, int index) {
		stringstream ss;
		ss << c << index;
		return ss.str();
	}

	/* ************************************************************************* */
	Pose2Config pose2Circle(size_t n, double R, char c) {
		Pose2Config x;
		double theta = 0, dtheta = 2*M_PI/n;
		for(size_t i=0;i<n;i++, theta+=dtheta)
			x.insert(symbol(c,i), Pose2(cos(theta), sin(theta), M_PI_2 + theta));
		return x;
	}

	/* ************************************************************************* */
} // namespace
