/*
 * NoiseModel.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */

#include "NoiseModel.h"

namespace ublas = boost::numeric::ublas;
typedef ublas::matrix_column<Matrix> column;

namespace gtsam {

	// functional
	Matrix GaussianNoiseModel::Whiten(const Matrix& H) const {
		size_t m = H.size1(), n = H.size2();
		Matrix W(m, n);
		for (int j = 0; j < n; j++) {
			Vector wj = whiten(column(H, j));
			for (int i = 0; i < m; i++)
				W(i, j) = wj(i);
		}
		return W;
	}

	// in place
	void GaussianNoiseModel::WhitenInPlace(Matrix& H) const {
		size_t m = H.size1(), n = H.size2();
		for (int j = 0; j < n; j++) {
			Vector wj = whiten(column(H, j));
			for (int i = 0; i < m; i++)
				H(i, j) = wj(i);
		}
	}

	Vector Isotropic::whiten(const Vector& v) const {
		return v * invsigma_;
	}

	Vector Isotropic::unwhiten(const Vector& v) const {
		return v * sigma_;
	}

	Diagonal::Diagonal(const Vector& sigmas) :
		GaussianNoiseModel(sigmas.size()), sigmas_(sigmas), invsigmas_(reciprocal(
				sigmas)) {
	}

	Vector Diagonal::whiten(const Vector& v) const {
		return emul(v, invsigmas_);
	}

	Vector Diagonal::unwhiten(const Vector& v) const {
		return emul(v, sigmas_);
	}

	static Vector sqrt(const Vector& v) {
		Vector s(v.size());
		transform(v.begin(), v.end(), s.begin(), ::sqrt);
		return s;
	}

	Variances::Variances(const Vector& variances) :
		Diagonal(sqrt(variances)) {
	}

	FullCovariance::FullCovariance(const Matrix& cov) :
		GaussianNoiseModel(cov.size1()),
				sqrt_covariance_(square_root_positive(cov)), sqrt_inv_covariance_(
						inverse_square_root(cov)) {
	}

	Vector FullCovariance::whiten(const Vector& v) const {
		return sqrt_inv_covariance_ * v;
	}

	Vector FullCovariance::unwhiten(const Vector& v) const {
		return sqrt_covariance_ * v;
	}

} // gtsam
