/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedKalmanFilter.h
 * @brief   Class to perform generic Kalman Filtering using nonlinear factor graphs
 * @author  Stephen Williams
 * @author  Chris Beall
 */

// \callgraph
#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

	/**
	 * This is a generic Extended Kalman Filter class implemented using nonlinear factors. GTSAM
	 * basically does SRIF with Cholesky to solve the filter problem, making this an efficient, numerically
	 * stable Kalman Filter implementation.
	 *
	 * The Kalman Filter relies on two models: a motion model that predicts the next state using
	 * the current state, and a measurement model that predicts the measurement value at a given
	 * state. Because these two models are situation-dependent, base classes for each have been
	 * provided above, from which the user must derive a class and incorporate the actual modeling
	 * equations.
	 *
	 * The class provides a "predict" and "update" function to perform these steps independently.
	 * TODO: a "predictAndUpdate" that combines both steps for some computational savings.
	 * \nosubgrouping
	 */

	template<class VALUE>
	class ExtendedKalmanFilter {
	public:

		typedef boost::shared_ptr<ExtendedKalmanFilter<VALUE> > shared_ptr;
		typedef VALUE T;
		typedef NoiseModelFactor2<VALUE, VALUE> MotionFactor;
		typedef NoiseModelFactor1<VALUE> MeasurementFactor;

	protected:
		T x_; // linearization point
		JacobianFactor::shared_ptr priorFactor_; // density

		T solve_(const GaussianFactorGraph& linearFactorGraph,
				const Ordering& ordering, const Values& linearizationPoints,
				Key x, JacobianFactor::shared_ptr& newPrior) const;

	public:

		/// @name Standard Constructors
		/// @{

		ExtendedKalmanFilter(T x_initial,
				noiseModel::Gaussian::shared_ptr P_initial);

		/// @}
		/// @name Testable
		/// @{

		/// print
	  void print(const std::string& s="") const {
	  	std::cout << s << "\n";
	  	x_.print(s+"x");
	  	priorFactor_->print(s+"density");
	  }

		/// @}
		/// @name Advanced Interface
		/// @{

	  ///TODO: comment
		T predict(const MotionFactor& motionFactor);

	  ///TODO: comment
		T update(const MeasurementFactor& measurementFactor);

		/// @}
	};

} // namespace

#include <gtsam/nonlinear/ExtendedKalmanFilter-inl.h>
