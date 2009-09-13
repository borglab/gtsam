// Frank Dellaert
// Simulated2D Prior

#include "NonlinearFactor.h"
#include "simulated2D.h"

namespace gtsam {

	class Point2Prior: public NonlinearFactor1 {
	public:
		Point2Prior(const Vector& mu, double sigma, const std::string& key) :
			NonlinearFactor1(mu, sigma, prior, key, Dprior) {
		}
	};

} // namespace gtsam
