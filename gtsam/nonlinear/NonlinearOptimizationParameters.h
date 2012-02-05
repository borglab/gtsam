/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearOptimizationParameters.h
 * @date Oct 14, 2010
 * @author Kai Ni
 * @brief structure to store parameters for nonlinear optimization
 */

#pragma once

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/nvp.hpp>

namespace gtsam {

	/**
	 *  a container for all related parameters
	 *  \nosubgrouping
	 */
	struct NonlinearOptimizationParameters {

		typedef boost::shared_ptr<NonlinearOptimizationParameters> shared_ptr;
		typedef NonlinearOptimizationParameters This;

		double absDecrease_; ///< threshold for the absolute decrease per iteration

		/**
		 * Relative decrease threshold, where relative error = (new-current)/current.
		 * This can be set to 0 if there is a possibility for negative error values.
		 */
		double relDecrease_;   ///< threshold for the relative decrease per iteration

		double sumError_;      ///< threshold for the sum of error
		size_t maxIterations_; ///< maximum number of iterations
		double lambda_;        ///< starting lambda value
		double lambdaFactor_;  ///< factor by which lambda is multiplied

		/// verbosity levels
		typedef enum {
			SILENT,
			ERROR,
			LAMBDA,
			TRYLAMBDA,
			VALUES,
			DELTA,
			TRYCONFIG,
			TRYDELTA,
			LINEAR,
			DAMPED
		} verbosityLevel;
		verbosityLevel verbosity_; ///< verbosity

		/// trust-region method mode
		typedef enum {
			FAST, BOUNDED, CAUTIOUS
		} LambdaMode;
		LambdaMode lambdaMode_; ///<trust-region method mode

		/// if true, solve whole system with QR, otherwise use LDL when possible
		bool useQR_;

		/// @name Standard Constructors
		/// @{

		/// Default constructor
		NonlinearOptimizationParameters();

		/// Constructor from doubles
		NonlinearOptimizationParameters(double absDecrease, double relDecrease,
				double sumError, int iIters = 100, double lambda = 1e-5,
				double lambdaFactor = 10, verbosityLevel v = SILENT,
				LambdaMode lambdaMode = BOUNDED, bool useQR = false);

		/// Copy constructor
		NonlinearOptimizationParameters(
				const NonlinearOptimizationParameters &parameters);

		/// @}
		/// @name Standard Interface
		/// @{

		/// print
		void print(const std::string& s="") const;

	  /// a copy of old instance except new lambda
		shared_ptr newLambda_(double lambda) const;

		/// @}
		/// @name Advanced Interface
		/// @{

		/// a copy of old instance except new verbosity
		static shared_ptr newVerbosity(verbosityLevel verbosity);

		/// a copy of old instance except new maxIterations
		static shared_ptr newMaxIterations(int maxIterations);

		/// a copy of old instance except new lambda
		static shared_ptr newLambda(double lambda);

		/// a copy of old instance except new thresholds
		static shared_ptr newDecreaseThresholds(double absDecrease,
				double relDecrease);

		/// a copy of old instance except new QR flag
		static shared_ptr newFactorization(bool useQR);

		/// @}

	private:
	  /** Serialization function */
	  friend class boost::serialization::access;
	  template<class ARCHIVE>
	  void serialize(ARCHIVE & ar, const unsigned int version)
	  {
	    ar & BOOST_SERIALIZATION_NVP(absDecrease_);
	    ar & BOOST_SERIALIZATION_NVP(relDecrease_);
	    ar & BOOST_SERIALIZATION_NVP(sumError_);
	    ar & BOOST_SERIALIZATION_NVP(maxIterations_);
	    ar & BOOST_SERIALIZATION_NVP(lambda_);
	    ar & BOOST_SERIALIZATION_NVP(lambdaFactor_);
	    ar & BOOST_SERIALIZATION_NVP(verbosity_);
	    ar & BOOST_SERIALIZATION_NVP(lambdaMode_);
	    ar & BOOST_SERIALIZATION_NVP(useQR_);
	  }
	};
}
