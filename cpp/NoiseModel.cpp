/*
 * NoiseModel.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */

//#include <boost/numeric/ublas/traits.hpp>

#include "NoiseModel.h"

namespace gtsam {

	Vector Isotropic::whiten(const Vector& v) const {
		return v * invsigma_;
	}

	Vector Isotropic::unwhiten(const Vector& v) const {
		return v * sigma_;
	}

	Diagonal::Diagonal(const Vector& sigmas) :
		sigmas_(sigmas), invsigmas_(1.0 / sigmas) {
	}

	Diagonal::Diagonal(const Diagonal& d) :
		sigmas_(d.sigmas_), invsigmas_(d.invsigmas_) {
	}

	Vector Diagonal::whiten(const Vector& v) const {
		return emul(v, invsigmas_);
	}

	Vector Diagonal::unwhiten(const Vector& v) const {
		return emul(v, sigmas_);
	}

	Variances::Variances(const Vector& variances) {
		sigmas_.resize(variances.size());
		std::transform(variances.begin(), variances.end(), sigmas_.begin(), sqrt);
		invsigmas_ = reciprocal(sigmas_);
	}

	FullCovariance::FullCovariance(const Matrix& cov) :
		sqrt_covariance_(square_root_positive(cov)), sqrt_inv_covariance_(
				inverse_square_root(cov)) {
	}

	FullCovariance::FullCovariance(const FullCovariance& cov) :
		sqrt_covariance_(cov.sqrt_covariance_), sqrt_inv_covariance_(
				cov.sqrt_inv_covariance_) {
	}

	Vector FullCovariance::whiten(const Vector& v) const {
		return sqrt_inv_covariance_ * v;
	}

	Vector FullCovariance::unwhiten(const Vector& v) const {
		return sqrt_covariance_ * v;
	}

} // gtsam
