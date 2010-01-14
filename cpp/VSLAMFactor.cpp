/**
 * @file    VSLAMFactor.cpp
 * @brief   A non-linear factor specialized to the Visual SLAM problem
 * @author  Alireza Fathi
 */

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include "VSLAMFactor.h"
#include "SimpleCamera.h"

using namespace std;
namespace gtsam {

	template class NonlinearFactor2<VSLAMConfig, VSLAMPoseKey, Pose3, VSLAMPointKey, Point3>;

	/* ************************************************************************* */
	VSLAMFactor::VSLAMFactor() {
		/// Arbitrary values
		K_ = shared_ptrK(new Cal3_S2(444, 555, 666, 777, 888));
	}
	/* ************************************************************************* */
	VSLAMFactor::VSLAMFactor(const Point2& z, double sigma, int cn, int ln,
			const shared_ptrK &K) :
		VSLAMFactorBase(sigma, cn, ln), z_(z), K_(K) {
	}

	/* ************************************************************************* */
	void VSLAMFactor::print(const std::string& s) const {
		VSLAMFactorBase::print(s);
		z_.print(s + ".z");
	}

	/* ************************************************************************* */
	bool VSLAMFactor::equals(const VSLAMFactor& p, double tol) const {
		return VSLAMFactorBase::equals(p, tol) && z_.equals(p.z_, tol)
				&& K_->equals(*p.K_, tol);
	}

/* ************************************************************************* */
} // namespace gtsam
