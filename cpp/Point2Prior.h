// Frank Dellaert
// Simulated2D Prior

#include "NonlinearFactor.h"
#include "simulated2D.h"

namespace simulated2D {

	struct Point2Prior: public gtsam::NonlinearFactor1<VectorConfig, std::string, Vector> {

		Vector z_;

		Point2Prior(const Vector& z, double sigma, const std::string& key) :
			gtsam::NonlinearFactor1<VectorConfig, std::string, Vector>(sigma, key), z_(z) {
		}

	  Vector evaluateError(const Vector& x, boost::optional<Matrix&> H = boost::none) const {
	    if (H) *H = Dprior(x);
	    return prior(x) - z_;
	  }

	};

} // namespace simulated2D
