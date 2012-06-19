/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace gtsam { // note, deliberately not in noiseModel namespace

	/**
	 * Just a convenient class to generate shared pointers to a noise model
	 */
  struct SharedNoiseModel: public noiseModel::Base::shared_ptr {

    typedef noiseModel::Base::shared_ptr Base;

  	/// @name Standard Constructors
  	/// @{

    SharedNoiseModel() {}
    SharedNoiseModel(const noiseModel::Robust::shared_ptr& p): Base(p) {}
    SharedNoiseModel(const noiseModel::Gaussian::shared_ptr& p): Base(p) {}
    SharedNoiseModel(const noiseModel::Diagonal::shared_ptr& p): Base(p) {}
    SharedNoiseModel(const noiseModel::Constrained::shared_ptr& p): Base(p) {}
    SharedNoiseModel(const noiseModel::Isotropic::shared_ptr& p): Base(p) {}
    SharedNoiseModel(const noiseModel::Unit::shared_ptr& p): Base(p) {}

    /// @}
    /// @name Dangerous constructors
    /// @{

    #ifdef GTSAM_MAGIC_GAUSSIAN
    SharedNoiseModel(const Matrix& covariance) :
      Base(boost::static_pointer_cast<noiseModel::Base>(
             noiseModel::Gaussian::Covariance(covariance))) {}

    SharedNoiseModel(const Vector& sigmas) :
      Base(boost::static_pointer_cast<noiseModel::Base>(
             noiseModel::Diagonal::Sigmas(sigmas))) {}
    #endif

    // Define GTSAM_DANGEROUS_GAUSSIAN to have access to bug-prone fixed dimension Gaussians
    // Not intended for human use, only for backwards compatibility of old unit tests
    #ifdef GTSAM_DANGEROUS_GAUSSIAN
    SharedNoiseModel(const double& s) :
      Base(boost::static_pointer_cast<noiseModel::Base>(
             noiseModel::Isotropic::Sigma(GTSAM_DANGEROUS_GAUSSIAN, s))) {}
    #endif

    /// @}
    /// @name Print
    /// @{

		/// Print
		inline void print(const std::string &s) const { (*this)->print(s); }

    /// @}
    /// @name Static syntactic sugar (should only be used with the MATLAB interface)
    /// @{

  	static inline SharedNoiseModel SqrtInformation(const Matrix& R) {
  	  return noiseModel::Gaussian::SqrtInformation(R);
  	}

  	static inline SharedNoiseModel Covariance(const Matrix& covariance, bool smart=false) {
  		return noiseModel::Gaussian::Covariance(covariance, smart);
  	}

  	static inline SharedNoiseModel Sigmas(const Vector& sigmas, bool smart=false) {
  		return noiseModel::Diagonal::Sigmas(sigmas, smart);
  	}

  	static inline SharedNoiseModel Variances(const Vector& variances, bool smart=false) {
  		return noiseModel::Diagonal::Variances(variances, smart);
  	}

  	static inline SharedNoiseModel Precisions(const Vector& precisions, bool smart=false) {
  	  return noiseModel::Diagonal::Precisions(precisions, smart);
  	}

  	static inline SharedNoiseModel Sigma(size_t dim, double sigma, bool smart=false) {
  		return noiseModel::Isotropic::Sigma(dim, sigma, smart);
  	}

  	static inline SharedNoiseModel Variance(size_t dim, double variance, bool smart=false) {
  		return noiseModel::Isotropic::Variance(dim, variance, smart);
  	}

  	static inline SharedNoiseModel Precision(size_t dim, double precision, bool smart=false) {
  	  return noiseModel::Isotropic::Precision(dim, precision, smart);
  	}

  	static inline SharedNoiseModel Unit(size_t dim) {
  	  return noiseModel::Unit::Create(dim);
  	}

    /// @}
  	/// @name Serialization
  	/// @{

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("SharedNoiseModel",
             boost::serialization::base_object<Base>(*this));
    }

    /// @}

  };
}
