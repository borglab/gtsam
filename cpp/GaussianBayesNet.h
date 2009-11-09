/**
 * @file    GaussianBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   GaussianBayesNet
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>

#include "ConditionalGaussian.h"
#include "BayesNet.h"

namespace gtsam {

	/** A Bayes net made from linear-Gaussian densities */
	typedef BayesNet<ConditionalGaussian> GaussianBayesNet;

	/** Create a scalar Gaussian */
	GaussianBayesNet scalarGaussian(const std::string& key, double mu=0.0, double sigma=1.0);

	/** Create a simple Gaussian on a single multivariate variable */
	GaussianBayesNet simpleGaussian(const std::string& key, const Vector& mu, double sigma=1.0);

	/**
	 * optimize, i.e. return x = inv(R)*d
	 */
	VectorConfig optimize(const GaussianBayesNet&);

	/**
	 * Return (dense) upper-triangular matrix representation
	 */
	std::pair<Matrix, Vector> matrix(const GaussianBayesNet&);

} /// namespace gtsam
