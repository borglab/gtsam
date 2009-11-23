/*
 * @file NonlinearConstraint-inl.h
 * @brief Implementation for NonlinearConstraints
 * @author alexgc
 */

#pragma once

#include <iostream>
#include "NonlinearConstraint.h"

namespace gtsam {

/* ************************************************************************* */
// Implementations of unary nonlinear constraints
/* ************************************************************************* */

template <class Config>
NonlinearConstraint1<Config>::NonlinearConstraint1(
			const std::string& key,
			Matrix (*gradG)(const Config& config, const std::string& key),
			Vector (*g)(const Config& config, const std::string& key),
			size_t dim_constraint,
			const std::string& lagrange_key) :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint),
				g_(g), gradG_(gradG), key_(key) {
		// set a good lagrange key here
		// TODO:should do something smart to find a unique one
		if (lagrange_key == "")
			this->lagrange_key_ = "L_" + key;
	}

/* ************************************************************************* */
template <class Config>
void NonlinearConstraint1<Config>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint1 [" << s << "]:\n"
			<< "  key:        " << key_ << "\n"
			<< "  p:          " << this->p_ << "\n"
			<< "  lambda key: " << this->lagrange_key_ << std::endl;
}

/* ************************************************************************* */
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

/* ************************************************************************* */
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
	GaussianFactor::shared_ptr constraint(new GaussianFactor(key_, grad, g, 0.0));

	return std::make_pair(factor, constraint);
}

/* ************************************************************************* */
// Implementations of binary nonlinear constraints
/* ************************************************************************* */

/* ************************************************************************* */
template <class Config>
NonlinearConstraint2<Config>::NonlinearConstraint2(
		const std::string& key1,
		Matrix (*gradG1)(const Config& config, const std::string& key),
		const std::string& key2,
		Matrix (*gradG2)(const Config& config, const std::string& key),
		Vector (*g)(const Config& config, const std::string& key1, const std::string& key2),
		size_t dim_constraint,
		const std::string& lagrange_key) :
			NonlinearConstraint<Config>(lagrange_key, dim_constraint),
			g_(g), gradG1_(gradG1), gradG2_(gradG2), key1_(key1), key2_(key2) {
	// set a good lagrange key here
	// TODO:should do something smart to find a unique one
	if (lagrange_key == "")
		this->lagrange_key_ = "L_" + key1 + key2;
}

/* ************************************************************************* */
template <class Config>
void NonlinearConstraint2<Config>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint2 [" << s << "]:\n"
			<< "  key1:       " << key1_ << "\n"
			<< "  key2:       " << key2_ << "\n"
			<< "  p:          " << this->p_ << "\n"
			<< "  lambda key: " << this->lagrange_key_ << std::endl;
}

/* ************************************************************************* */
template <class Config>
bool NonlinearConstraint2<Config>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint2<Config>* p = dynamic_cast<const NonlinearConstraint2<Config>*> (&f);
	if (p == NULL) return false;
	if (key1_ != p->key1_) return false;
	if (key2_ != p->key2_) return false;
	if (this->lagrange_key_ != p->lagrange_key_) return false;
	if (g_ != p->g_) return false;
	if (gradG1_ != p->gradG1_) return false;
	if (gradG2_ != p->gradG2_) return false;
	return this->p_ == p->p_;
}

/* ************************************************************************* */
template <class Config>
std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr>
NonlinearConstraint2<Config>::linearize(const Config& config, const VectorConfig& lagrange) const {
	// extract lagrange multiplier
	Vector lambda = lagrange[this->lagrange_key_];

	// find the error
	Vector g = g_(config, key1_, key2_);

	// construct the gradients
	Matrix grad1 = gradG1_(config, key1_);
	Matrix grad2 = gradG2_(config, key2_);

	// construct probabilistic factor
	Matrix A1 = vector_scale(grad1, lambda);
	Matrix A2 = vector_scale(grad2, lambda);
	GaussianFactor::shared_ptr factor(new
			GaussianFactor(key1_, A1, key2_, A2,
					this->lagrange_key_, eye(this->p_), zero(this->p_), 1.0));

	// construct the constraint
	GaussianFactor::shared_ptr constraint(new GaussianFactor(key1_, grad1, key2_, grad2, g, 0.0));

	return std::make_pair(factor, constraint);
}

}
