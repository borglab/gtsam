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

	/* ************************************************************************* */
	Vector GaussianNoiseModel::whiten(const Vector& v) const {
		return sqrt_information_ * v;
	}

	Vector GaussianNoiseModel::unwhiten(const Vector& v) const {
		return backSubstituteUpper(sqrt_information_, v);
	}

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

	/* ************************************************************************* */
	// TODO: can we avoid calling reciprocal twice ?
	Diagonal::Diagonal(const Vector& sigmas) :
		GaussianNoiseModel(diag(reciprocal(sigmas))),
				invsigmas_(reciprocal(sigmas)), sigmas_(sigmas) {
	}

	Vector Diagonal::whiten(const Vector& v) const {
		return emul(v, invsigmas_);
	}

	Vector Diagonal::unwhiten(const Vector& v) const {
		return emul(v, sigmas_);
	}

	/* ************************************************************************* */

	Vector Isotropic::whiten(const Vector& v) const {
		return v * invsigma_;
	}

	Vector Isotropic::unwhiten(const Vector& v) const {
		return v * sigma_;
	}

/* ************************************************************************* */
} // gtsam
