/*
 * @file NonlinearConstraint-inl.h
 * @brief Implementation for NonlinearConstraints
 * @author alexgc
 */

#pragma once

#include <iostream>
#include "NonlinearConstraint.h"

namespace gtsam {

/**
 * Implementations of unary nonlinear constraints
 */

template <class Config>
void NonlinearConstraint1<Config>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint [" << s << "]:\n"
			<< "  Variable:   " << key_ << "\n"
			<< "  p:          " << this->p_ << "\n"
			<< "  lambda key: " << this->lagrange_key_ << std::endl;
}

template <class Config>
bool NonlinearConstraint1<Config>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint1<Config>* p = dynamic_cast<const NonlinearConstraint1<Config>*> (&f);
	if (p == NULL) return false;
	if (key_ != p->key_) return false;
	if (this->lagrange_key_ != p->lagrange_key_) return false;
	if (g_ != p->g_) return false;
	if (gradG_ != p->gradG_) return false;
	return this->p_ == p->p_;
}

template <class Config>
std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr>
NonlinearConstraint1<Config>::linearize(const Config& config, const VectorConfig& lagrange) const {
	// extract lagrange multiplier
	Vector lambda = lagrange[this->lagrange_key_];

	// find the error
	Vector g = g_(config, key_);

	// construct the gradient
	Matrix grad = gradG_(config, key_);

	// construct probabilistic factor
	Matrix A1 = vector_scale(grad, lambda);
	GaussianFactor::shared_ptr factor(new
			GaussianFactor(key_, A1, this->lagrange_key_, eye(this->p_), zero(this->p_), 1.0));

	// construct the constraint
	GaussianFactor::shared_ptr constraint(new GaussianFactor("x", grad, g, 0.0));

	return std::make_pair(factor, constraint);
}

}
