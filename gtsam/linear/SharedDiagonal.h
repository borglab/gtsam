/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * SharedDiagonal.h
 * @brief Class that wraps a shared noise model with diagonal covariance
 * @Author: Frank Dellaert
 * Created on: Jan 22, 2010
 */

#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace gtsam { // note, deliberately not in noiseModel namespace

	// A useful convenience class to refer to a shared Diagonal model
	// There are (somewhat dangerous) constructors from Vector and pair<size_t,double>
	// that call Sigmas and Sigma, respectively.
	struct SharedDiagonal: public noiseModel::Diagonal::shared_ptr {
		SharedDiagonal() {
		}
		SharedDiagonal(const noiseModel::Diagonal::shared_ptr& p) :
			noiseModel::Diagonal::shared_ptr(p) {
		}
		SharedDiagonal(const noiseModel::Constrained::shared_ptr& p) :
			noiseModel::Diagonal::shared_ptr(p) {
		}
		SharedDiagonal(const noiseModel::Isotropic::shared_ptr& p) :
			noiseModel::Diagonal::shared_ptr(p) {
		}
		SharedDiagonal(const noiseModel::Unit::shared_ptr& p) :
			noiseModel::Diagonal::shared_ptr(p) {
		}
		SharedDiagonal(const Vector& sigmas) :
			noiseModel::Diagonal::shared_ptr(noiseModel::Diagonal::Sigmas(sigmas)) {
		}

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("SharedDiagonal",
			        boost::serialization::base_object<noiseModel::Diagonal::shared_ptr >(*this));
		}
	};

	// TODO: make these the ones really used in unit tests
	inline SharedDiagonal sharedSigmas(const Vector& sigmas) {
		return noiseModel::Diagonal::Sigmas(sigmas);
	}
	inline SharedDiagonal sharedSigma(size_t dim, double sigma) {
		return noiseModel::Isotropic::Sigma(dim, sigma);
	}
	inline SharedDiagonal sharedPrecisions(const Vector& precisions) {
	  return noiseModel::Diagonal::Precisions(precisions);
	}
	inline SharedDiagonal sharedPrecision(size_t dim, double precision) {
	  return noiseModel::Isotropic::Precision(dim, precision);
	}

}

