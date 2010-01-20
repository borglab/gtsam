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
					Vector (*g)(const Config& config),
					bool isEquality)
:	NonlinearFactor<Config>(1.0),
	lagrange_key_(lagrange_key), p_(dim_lagrange),
	isEquality_(isEquality), g_(boost::bind(g, _1)) {}

/* ************************************************************************* */
template <class Config>
NonlinearConstraint<Config>::NonlinearConstraint(const std::string& lagrange_key,
					size_t dim_lagrange,
					boost::function<Vector(const Config& config)> g,
					bool isEquality)
:	NonlinearFactor<Config>(noiseModel::Constrained::All(dim_lagrange)),
	lagrange_key_(lagrange_key), p_(dim_lagrange),
	g_(g), isEquality_(isEquality) {}

/* ************************************************************************* */
template <class Config>
bool NonlinearConstraint<Config>::active(const Config& config) const {
	return !(!isEquality_ && greaterThanOrEqual(unwhitenedError(config), zero(p_)));
}

/* ************************************************************************* */
// Implementations of unary nonlinear constraints
/* ************************************************************************* */

template <class Config, class Key, class X>
NonlinearConstraint1<Config, Key, X>::NonlinearConstraint1(
			Vector (*g)(const Config& config),
			const Key& key,
			Matrix (*gradG)(const Config& config),
			size_t dim_constraint,
			const std::string& lagrange_key,
			bool isEquality) :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, g, isEquality),
				G_(boost::bind(gradG, _1)), key_(key)
{
		// set a good lagrange key here
		// TODO:should do something smart to find a unique one
//		if (lagrange_key == "")
//			this->lagrange_key_ = "L0"
//		this->keys_.push_front(key);
}

/* ************************************************************************* */
template <class Config, class Key, class X>
NonlinearConstraint1<Config, Key, X>::NonlinearConstraint1(
		boost::function<Vector(const Config& config)> g,
			const Key& key,
			boost::function<Matrix(const Config& config)> gradG,
			size_t dim_constraint,
			const std::string& lagrange_key,
			bool isEquality) :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, g, isEquality),
				G_(gradG), key_(key)
{
	// set a good lagrange key here
	// TODO:should do something smart to find a unique one
//	if (lagrange_key == "")
//		this->lagrange_key_ = "L_" + key;
//	this->keys_.push_front(key);
}

/* ************************************************************************* */
template <class Config, class Key, class X>
void NonlinearConstraint1<Config, Key, X>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint1 [" << s << "]:\n";
//			<< "  key:        " << key_ << "\n"
//			<< "  p:          " << this->p_ << "\n"
//			<< "  lambda key: " << this->lagrange_key_ << "\n";
//	if (this->isEquality_)
//		std::cout << "  Equality Factor" << std::endl;
//	else
//		std::cout << "  Inequality Factor" << std::endl;
}

/* ************************************************************************* */
template <class Config, class Key, class X>
bool NonlinearConstraint1<Config, Key, X>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint1<Config, Key, X>* p = dynamic_cast<const NonlinearConstraint1<Config, Key, X>*> (&f);
	if (p == NULL) return false;
	if (!(key_ == p->key_)) return false;
	if (this->lagrange_key_ != p->lagrange_key_) return false;
	if (this->isEquality_ != p->isEquality_) return false;
	return this->p_ == p->p_;
}

/* ************************************************************************* */
template <class Config, class Key, class X>
std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr>
NonlinearConstraint1<Config, Key, X>::linearize(const Config& config, const VectorConfig& lagrange) const {
	// extract lagrange multiplier
	Vector lambda = lagrange[this->lagrange_key_];

	// find the error
	Vector g = g_(config);

	// construct the gradient
	Matrix grad = G_(config);

	// construct probabilistic factor
	Matrix A1 = vector_scale(lambda, grad);
	sharedDiagonal probModel = sharedSigma(this->p_,1.0);
	GaussianFactor::shared_ptr factor(new
			GaussianFactor(key_, A1, this->lagrange_key_, eye(this->p_), zero(this->p_), probModel));

	// construct the constraint
	sharedDiagonal constraintModel = noiseModel::Constrained::All(this->p_);
	GaussianFactor::shared_ptr constraint(new GaussianFactor(key_, grad, -1*g, constraintModel));

	return std::make_pair(factor, constraint);
}

/* ************************************************************************* */
// Implementations of binary nonlinear constraints
/* ************************************************************************* */

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
NonlinearConstraint2<Config, Key1, X1, Key2, X2>::NonlinearConstraint2(
		Vector (*g)(const Config& config),
		const Key1& key1,
		Matrix (*G1)(const Config& config),
		const Key2& key2,
		Matrix (*G2)(const Config& config),
		size_t dim_constraint,
		const std::string& lagrange_key,
		bool isEquality) :
			NonlinearConstraint<Config>(lagrange_key, dim_constraint, g, isEquality),
			G1_(boost::bind(G1, _1)), G2_(boost::bind(G2, _1)),
			key1_(key1), key2_(key2)
{
	// set a good lagrange key here
	// TODO:should do something smart to find a unique one
//	if (lagrange_key == "")
//		this->lagrange_key_ = "L_" + key1 + key2;
//	this->keys_.push_front(key1);
//	this->keys_.push_back(key2);
}

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
NonlinearConstraint2<Config, Key1, X1, Key2, X2>::NonlinearConstraint2(
		boost::function<Vector(const Config& config)> g,
		const Key1& key1,
		boost::function<Matrix(const Config& config)> G1,
		const Key2& key2,
		boost::function<Matrix(const Config& config)> G2,
		size_t dim_constraint,
		const std::string& lagrange_key,
		bool isEquality)  :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, g, isEquality),
				G1_(G1), G2_(G2),
				key1_(key1), key2_(key2)
{
	// set a good lagrange key here
	// TODO:should do something smart to find a unique one
//	if (lagrange_key == "")
//		this->lagrange_key_ = "L_" + key1 + key2;
//	this->keys_.push_front(key1);
//	this->keys_.push_back(key2);
}

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
void NonlinearConstraint2<Config, Key1, X1, Key2, X2>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint2 [" << s << "]:\n";
//			<< "  key1:       " << key1_ << "\n"
//			<< "  key2:       " << key2_ << "\n"
//			<< "  p:          " << this->p_ << "\n"
//			<< "  lambda key: " << this->lagrange_key_ << std::endl;
}

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
bool NonlinearConstraint2<Config, Key1, X1, Key2, X2>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint2<Config, Key1, X1, Key2, X2>* p = dynamic_cast<const NonlinearConstraint2<Config, Key1, X1, Key2, X2>*> (&f);
	if (p == NULL) return false;
	if (!(key1_ == p->key1_)) return false;
	if (!(key2_ == p->key2_)) return false;
	if (this->lagrange_key_ != p->lagrange_key_) return false;
	if (this->isEquality_ != p->isEquality_) return false;
	return this->p_ == p->p_;
}

/* ************************************************************************* */
template<class Config, class Key1, class X1, class Key2, class X2>
std::pair<GaussianFactor::shared_ptr, GaussianFactor::shared_ptr> NonlinearConstraint2<
		Config, Key1, X1, Key2, X2>::linearize(const Config& config, const VectorConfig& lagrange) const {
	// extract lagrange multiplier
	Vector lambda = lagrange[this->lagrange_key_];

	// find the error
	Vector g = g_(config);

	// construct the gradients
	Matrix grad1 = G1_(config);
	Matrix grad2 = G2_(config);

	// construct probabilistic factor
	Matrix A1 = vector_scale(lambda, grad1);
	Matrix A2 = vector_scale(lambda, grad2);
	sharedDiagonal probModel = sharedSigma(this->p_,1.0);
	GaussianFactor::shared_ptr factor(new GaussianFactor(key1_, A1, key2_, A2,
			this->lagrange_key_, eye(this->p_), zero(this->p_), probModel));

	// construct the constraint
	sharedDiagonal constraintModel = noiseModel::Constrained::All(this->p_);
	GaussianFactor::shared_ptr constraint(new GaussianFactor(key1_, grad1,
			key2_, grad2, -1.0 * g, constraintModel));

	return std::make_pair(factor, constraint);
}

}
