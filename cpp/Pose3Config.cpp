/**
 * @file    Pose3Config.cpp
 * @brief   Configuration of 3D poses
 * @author  Frank Dellaert
 */

#include "Pose3Config.h"
#include "LieConfig-inl.h"

using namespace std;

namespace gtsam {

  /** Explicit instantiation of templated methods and functions */
  template class LieConfig<Pose3>;
  template size_t dim(const LieConfig<Pose3>& c);
  template LieConfig<Pose3> expmap(const LieConfig<Pose3>& c, const Vector& delta);
  template LieConfig<Pose3> expmap(const LieConfig<Pose3>& c, const VectorConfig& delta);

	/* ************************************************************************* */
	// TODO: local version, should probably defined in LieConfig
	static string symbol(char c, int index) {
		stringstream ss;
		ss << c << index;
		return ss.str();
	}

	/* ************************************************************************* */
	Pose3Config pose3Circle(size_t n, double R, char c) {
		Pose3Config x;
		double theta = 0, dtheta = 2*M_PI/n;
		// Vehicle at p0 is looking towards y axis
		Rot3 R0(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
		for (size_t i = 0; i < n; i++, theta += dtheta)
			x.insert(symbol(c,i), Pose3(R0 * Rot3::yaw(-theta), Point3(cos(theta),sin(theta),0)));
		return x;
	}

	/* ************************************************************************* */
} // namespace
