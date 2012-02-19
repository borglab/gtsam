/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedKalmanFilter-inl.h
 * @brief   Class to perform generic Kalman Filtering using nonlinear factor graphs
 * @author  Stephen Williams
 * @author  Chris Beall
 */

#pragma once

#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

	/* ************************************************************************* */
	template<class VALUE>
	typename ExtendedKalmanFilter<VALUE>::T ExtendedKalmanFilter<VALUE>::solve_(
			const GaussianFactorGraph& linearFactorGraph, const Ordering& ordering,
			const Values& linearizationPoints, Key lastKey,
			JacobianFactor::shared_ptr& newPrior) const {

		// Extract the Index of the provided last key
		gtsam::Index lastIndex = ordering.at(lastKey);

		// Solve the linear factor graph, converting it into a linear Bayes Network
		// P(x0,x1) = P(x0|x1)*P(x1)
		GaussianSequentialSolver solver(linearFactorGraph);
		GaussianBayesNet::shared_ptr linearBayesNet = solver.eliminate();

		// Extract the current estimate of x1,P1 from the Bayes Network
		VectorValues result = optimize(*linearBayesNet);
		T x = linearizationPoints.at<T>(lastKey).retract(result[lastIndex]);

		// Create a Jacobian Factor from the root node of the produced Bayes Net.
		// This will act as a prior for the next iteration.
		// The linearization point of this prior must be moved to the new estimate of x,
		// and the key/index needs to be reset to 0, the first key in the next iteration.
		const GaussianConditional::shared_ptr& cg = linearBayesNet->back();
		assert(cg->nrFrontals() == 1);
		assert(cg->nrParents() == 0);
		// TODO: Find a way to create the correct Jacobian Factor in a single pass
		JacobianFactor tmpPrior = JacobianFactor(*cg);
		newPrior = JacobianFactor::shared_ptr(
				new JacobianFactor(
						0,
						tmpPrior.getA(tmpPrior.begin()),
						tmpPrior.getb()
								- tmpPrior.getA(tmpPrior.begin()) * result[lastIndex],
						tmpPrior.get_model()));

		return x;
	}

	/* ************************************************************************* */
	template<class VALUE>
	ExtendedKalmanFilter<VALUE>::ExtendedKalmanFilter(T x_initial,
			noiseModel::Gaussian::shared_ptr P_initial) {

		// Set the initial linearization point to the provided mean
		x_ = x_initial;

		// Create a Jacobian Prior Factor directly P_initial.
		// Since x0 is set to the provided mean, the b vector in the prior will be zero
		// TODO Frank asks: is there a reason why noiseModel is not simply P_initial ?
		priorFactor_ = JacobianFactor::shared_ptr(
				new JacobianFactor(0, P_initial->R(), Vector::Zero(x_initial.dim()),
						noiseModel::Unit::Create(P_initial->dim())));
	}

	/* ************************************************************************* */
	template<class VALUE>
	typename ExtendedKalmanFilter<VALUE>::T ExtendedKalmanFilter<VALUE>::predict(
			const MotionFactor& motionFactor) {

		// TODO: This implementation largely ignores the actual factor symbols.
		// Calling predict() then update() with drastically
		// different keys will still compute as if a common key-set was used

		// Create Keys
		Key x0 = motionFactor.key1();
		Key x1 = motionFactor.key2();

		// Create an elimination ordering
		Ordering ordering;
		ordering.insert(x0, 0);
		ordering.insert(x1, 1);

		// Create a set of linearization points
		Values linearizationPoints;
		linearizationPoints.insert(x0, x_);
		linearizationPoints.insert(x1, x_); // TODO should this really be x_ ?

		// Create a Gaussian Factor Graph
		GaussianFactorGraph linearFactorGraph;

		// Add in previous posterior as prior on the first state
		linearFactorGraph.push_back(priorFactor_);

		// Linearize motion model and add it to the Kalman Filter graph
		linearFactorGraph.push_back(
				motionFactor.linearize(linearizationPoints, ordering));

		// Solve the factor graph and update the current state estimate
		// and the posterior for the next iteration.
		x_ = solve_(linearFactorGraph, ordering, linearizationPoints, x1, priorFactor_);

		return x_;
	}

	/* ************************************************************************* */
	template<class VALUE>
	typename ExtendedKalmanFilter<VALUE>::T ExtendedKalmanFilter<VALUE>::update(
			const MeasurementFactor& measurementFactor) {

		// TODO: This implementation largely ignores the actual factor symbols.
		// Calling predict() then update() with drastically
		// different keys will still compute as if a common key-set was used

		// Create Keys
		Key x0 = measurementFactor.key();

		// Create an elimination ordering
		Ordering ordering;
		ordering.insert(x0, 0);

		// Create a set of linearization points
		Values linearizationPoints;
		linearizationPoints.insert(x0, x_);

		// Create a Gaussian Factor Graph
		GaussianFactorGraph linearFactorGraph;

		// Add in the prior on the first state
		linearFactorGraph.push_back(priorFactor_);

		// Linearize measurement factor and add it to the Kalman Filter graph
		linearFactorGraph.push_back(
				measurementFactor.linearize(linearizationPoints, ordering));

		// Solve the factor graph and update the current state estimate
		// and the prior factor for the next iteration
		x_ = solve_(linearFactorGraph, ordering, linearizationPoints, x0, priorFactor_);

		return x_;
	}

} // namespace gtsam
