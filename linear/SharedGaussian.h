/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * SharedGaussian.h
 * @brief Class that wraps a shared noise model with diagonal covariance
 * @Author: Frank Dellaert
 * Created on: Jan 22, 2010
 */

#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace gtsam { // note, deliberately not in noiseModel namespace

	/**
	 * A useful convenience class to refer to a shared Gaussian model
	 * Also needed to make noise models in matlab
	 */
	struct SharedGaussian: public noiseModel::Gaussian::shared_ptr {
		SharedGaussian() {
		}
		// TODO: better way ?
		SharedGaussian(const noiseModel::Gaussian::shared_ptr& p) :
			noiseModel::Gaussian::shared_ptr(p) {
		}
		SharedGaussian(const noiseModel::Diagonal::shared_ptr& p) :
			noiseModel::Gaussian::shared_ptr(p) {
		}
		SharedGaussian(const noiseModel::Constrained::shared_ptr& p) :
			noiseModel::Gaussian::shared_ptr(p) {
		}
		SharedGaussian(const noiseModel::Isotropic::shared_ptr& p) :
			noiseModel::Gaussian::shared_ptr(p) {
		}
		SharedGaussian(const noiseModel::Unit::shared_ptr& p) :
			noiseModel::Gaussian::shared_ptr(p) {
		}

		// Define GTSAM_MAGIC_GAUSSIAN to have access to slightly
		// dangerous and non-shared (inefficient, wasteful) noise models.
		// Intended to be used only in tests (if you must) and the MATLAB wrapper
#ifdef GTSAM_MAGIC_GAUSSIAN
		SharedGaussian(const Matrix& covariance) :
			noiseModel::Gaussian::shared_ptr(noiseModel::Gaussian::Covariance(covariance)) {
		}
		SharedGaussian(const Vector& sigmas) :
			noiseModel::Gaussian::shared_ptr(noiseModel::Diagonal::Sigmas(sigmas)) {
		}
#endif
// Define GTSAM_DANGEROUS_GAUSSIAN to have access to bug-prone fixed dimension Gaussians
// Not intended for human use, only for backwards compatibility of old unit tests
#ifdef GTSAM_DANGEROUS_GAUSSIAN
		SharedGaussian(const double& s) :
			noiseModel::Gaussian::shared_ptr(noiseModel::Isotropic::Sigma(
					GTSAM_DANGEROUS_GAUSSIAN, s)) {
		}
#endif
	};

}
