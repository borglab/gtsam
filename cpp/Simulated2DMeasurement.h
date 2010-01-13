// Christian Potthast
// Simulated2D Odometry

#include "NonlinearFactor.h"
#include "simulated2D.h"

namespace simulated2D {

	struct Simulated2DMeasurement: public gtsam::NonlinearFactor2<VectorConfig,
			PoseKey, Vector, PointKey, Vector> {

		Vector z_;

		Simulated2DMeasurement(const Vector& z, double sigma, const std::string& j1,
				const std::string& j2) :
			z_(z), gtsam::NonlinearFactor2<VectorConfig, PoseKey, Vector, PointKey,
					Vector>(sigma, j1, j2) {
		}

		Vector evaluateError(const Vector& x1, const Vector& x2, boost::optional<
				Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
			if (H1) *H1 = Dmea1(x1, x2);
			if (H2) *H2 = Dmea2(x1, x2);
			return mea(x1, x2) - z_;
		}

	};

} // namespace simulated2D
