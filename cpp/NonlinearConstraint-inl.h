/*
 * @file NonlinearConstraint-inl.h
 * @brief Implementation for NonlinearConstraints
 * @author Alex Cunningham
 */

#pragma once

#include <iostream>
#include <boost/bind.hpp>
#include "NonlinearConstraint.h"

namespace gtsam {

/* ************************************************************************* */
// Implementations of base class
/* ************************************************************************* */

/* ************************************************************************* */
template <class Config>
NonlinearConstraint<Config>::NonlinearConstraint(const std::string& lagrange_key,
					size_t dim_lagrange,
					Vector (*g)(const Config& config, const std::list<std::string>& keys),
					bool isEquality)
:	NonlinearFactor<Config>(zero(dim_lagrange), 1.0),
	lagrange_key_(lagrange_key), p_(dim_lagrange),
	isEquality_(isEquality), g_(boost::bind(g, _1, _2)) {}

/* ************************************************************************* */
template <class Config>
NonlinearConstraint<Config>::NonlinearConstraint(const std::string& lagrange_key,
					size_t dim_lagrange,
					boost::function<Vector(const Config& config, const std::list<std::string>& keys)> g,
					bool isEquality)
:	NonlinearFactor<Config>(zero(dim_lagrange), 1.0),
	lagrange_key_(lagrange_key), p_(dim_lagrange),
	g_(g), isEquality_(isEquality) {}

/* ************************************************************************* */
template <class Config>
bool NonlinearConstraint<Config>::active(const Config& config) const {
	return !(!isEquality_ && greaterThanOrEqual(error_vector(config), zero(p_)));
}

/* ************************************************************************* */
// Implementations of unary nonlinear constraints
/* ************************************************************************* */

template <class Config>
NonlinearConstraint1<Config>::NonlinearConstraint1(
			const std::string& key,
			Matrix (*gradG)(const Config& config, const std::list<std::string>& keys),
			Vector (*g)(const Config& config, const std::list<std::string>& keys),
			size_t dim_constraint,
			const std::string& lagrange_key,
			bool isEquality) :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, g, isEquality),
				gradG_(boost::bind(gradG, _1, _2)), key_(key)
{
		// set a good lagrange key here
		// TODO:should do something smart to find a unique one
		if (lagrange_key == "")
			this->lagrange_key_ = "L_" + key;
		this->keys_.push_front(key);
}

/* ************************************************************************* */
template <class Config>
NonlinearConstraint1<Config>::NonlinearConstraint1(
			const std::string& key,
			boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG,
			boost::function<Vector(const Config& config, const std::list<std::string>& keys)> g,
			size_t dim_constraint,
			const std::string& lagrange_key,
			bool isEquality) :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, g, isEquality),
				gradG_(gradG), key_(key)
{
	// set a good lagrange key here
	// TODO:should do something smart to find a unique one
	if (lagrange_key == "")
		this->lagrange_key_ = "L_" + key;
	this->keys_.push_front(key);
}

/* ************************************************************************* */
template <class Config>
void NonlinearConstraint1<Config>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint1 [" << s << "]:\n"
			<< "  key:        " << key_ << "\n"
			<< "  p:          " << this->p_ << "\n"
			<< "  lambda key: " << this->lagrange_key_ << "\n";
	if (this->isEquality_)
		std::cout << "  Equality Factor" << std::endl;
	else
		std::cout << "  Inequality Factor" << std::endl;
}

/* ************************************************************************* */
template <class Config>
bool NonlinearConstraint1<Config>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint1<Config>* p = dynamic_cast<const NonlinearConstraint1<Config>*> (&f);
	if (p == NULL) return false;
	if (key_ != p->key_) return false;
	if (this->lagrange_key_ != p->lagrange_key_) return false;
	if (this->isEquality_ != p->isEquality_) return false;
	return this->p_ == p->p_;
}

/* ************************************************************************* */
template <class Config>
std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr>
NonlinearConstraint1<Config>::linearize(const Config& config, const VectorConfig& lagrange) const {
	// extract lagrange multiplier
	Vector lambda = lagrange[this->lagrange_key_];

	// find the error
	Vector g = g_(config, this->keys());

	// construct the gradient
	Matrix grad = gradG_(config, this->keys());

	// construct probabilistic factor
	Matrix A1 = vector_scale(lambda, grad);
	GaussianFactor::shared_ptr factor(new
			GaussianFactor(key_, A1, this->lagrange_key_, eye(this->p_), zero(this->p_), 1.0));

	// construct the constraint
	GaussianFactor::shared_ptr constraint(new GaussianFactor(key_, grad, -1*g, 0.0));

	return std::make_pair(factor, constraint);
}

/* ************************************************************************* */
// Implementations of binary nonlinear constraints
/* ************************************************************************* */

/* ************************************************************************* */
template <class Config>
NonlinearConstraint2<Config>::NonlinearConstraint2(
		const std::string& key1,
		Matrix (*gradG1)(const Config& config, const std::list<std::string>& keys),
		const std::string& key2,
		Matrix (*gradG2)(const Config& config, const std::list<std::string>& keys),
		Vector (*g)(const Config& config, const std::list<std::string>& keys),
		size_t dim_constraint,
		const std::string& lagrange_key,
		bool isEquality) :
			NonlinearConstraint<Config>(lagrange_key, dim_constraint, g, isEquality),
			gradG1_(boost::bind(gradG1, _1, _2)), gradG2_(boost::bind(gradG2, _1, _2)),
			key1_(key1), key2_(key2)
{
	// set a good lagrange key here
	// TODO:should do something smart to find a unique one
	if (lagrange_key == "")
		this->lagrange_key_ = "L_" + key1 + key2;
	this->keys_.push_front(key1);
	this->keys_.push_back(key2);
}

/* ************************************************************************* */
template <class Config>
NonlinearConstraint2<Config>::NonlinearConstraint2(
		const std::string& key1,
		boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG1,
		const std::string& key2,
		boost::function<Matrix(const Config& config, const std::list<std::string>& keys)> gradG2,
		boost::function<Vector(const Config& config, const std::list<std::string>& keys)> g,
		size_t dim_constraint,
		const std::string& lagrange_key,
		bool isEquality)  :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, g, isEquality),
				gradG1_(gradG1), gradG2_(gradG2),
				key1_(key1), key2_(key2)
{
	// set a good lagrange key here
	// TODO:should do something smart to find a unique one
	if (lagrange_key == "")
		this->lagrange_key_ = "L_" + key1 + key2;
	this->keys_.push_front(key1);
	this->keys_.push_back(key2);
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
	if (this->isEquality_ != p->isEquality_) return false;
	return this->p_ == p->p_;
}

/* ************************************************************************* */
template <class Config>
std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr>
NonlinearConstraint2<Config>::linearize(const Config& config, const VectorConfig& lagrange) const {
	// extract lagrange multiplier
	Vector lambda = lagrange[this->lagrange_key_];

	// find the error
	Vector g = g_(config, this->keys());

	// construct the gradients
	Matrix grad1 = gradG1_(config, this->keys());
	Matrix grad2 = gradG2_(config, this->keys());

	// construct probabilistic factor
	Matrix A1 = vector_scale(lambda, grad1);
	Matrix A2 = vector_scale(lambda, grad2);
	GaussianFactor::shared_ptr factor(new
			GaussianFactor(key1_, A1, key2_, A2,
					this->lagrange_key_, eye(this->p_), zero(this->p_), 1.0));

	// construct the constraint
	GaussianFactor::shared_ptr constraint(new GaussianFactor(key1_, grad1, key2_, grad2, -1.0*g, 0.0));

	return std::make_pair(factor, constraint);
}

}
