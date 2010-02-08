/*
 * @file NonlinearConstraint-inl.h
 * @brief Implementation for NonlinearConstraints
 * @author Alex Cunningham
 */

#pragma once

#include <iostream>
#include <boost/bind.hpp>
#include "NonlinearConstraint.h"

#define INSTANTIATE_NONLINEAR_CONSTRAINT(C) \
  INSTANTIATE_FACTOR_GRAPH(NonlinearConstraint<C>); \
  template class NonlinearConstraint<C>;

namespace gtsam {

/* ************************************************************************* */
// Implementations of base class
/* ************************************************************************* */

/* ************************************************************************* */
template <class Config>
NonlinearConstraint<Config>::NonlinearConstraint(const LagrangeKey& lagrange_key,
					size_t dim_lagrange,
					bool isEquality) :
	NonlinearFactor<Config>(noiseModel::Constrained::All(dim_lagrange)),
	lagrange_key_(lagrange_key), p_(dim_lagrange), isEquality_(isEquality) {}

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
			const LagrangeKey& lagrange_key,
			bool isEquality) :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, isEquality),
				g_(boost::bind(g, _1)), G_(boost::bind(gradG, _1)), key_(key)
{
}

/* ************************************************************************* */
template <class Config, class Key, class X>
NonlinearConstraint1<Config, Key, X>::NonlinearConstraint1(
		boost::function<Vector(const Config& config)> g,
			const Key& key,
			boost::function<Matrix(const Config& config)> gradG,
			size_t dim_constraint,
			const LagrangeKey& lagrange_key,
			bool isEquality) :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, isEquality),
				g_(g), G_(gradG), key_(key)
{
}

/* ************************************************************************* */
template <class Config, class Key, class X>
void NonlinearConstraint1<Config, Key, X>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint1 [" << s << "]: Dim: " << this->p_ << "\n"
			  << "  Key         : " << (std::string) this->key_ << "\n"
			  << "  Lagrange Key: " << (std::string) this->lagrange_key_ << "\n";
	if (this->isEquality_)
		std::cout << "  Equality Factor" << std::endl;
	else
		std::cout << "  Inequality Factor" << std::endl;
}

/* ************************************************************************* */
template <class Config, class Key, class X>
bool NonlinearConstraint1<Config, Key, X>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint1<Config, Key, X>* p = dynamic_cast<const NonlinearConstraint1<Config, Key, X>*> (&f);
	if (p == NULL) return false;
	if (!(key_ == p->key_)) return false;
	if (!(this->lagrange_key_.equals(p->lagrange_key_))) return false;
	if (this->isEquality_ != p->isEquality_) return false;
	return this->p_ == p->p_;
}

/* ************************************************************************* */
template <class Config, class Key, class X>
GaussianFactor::shared_ptr
NonlinearConstraint1<Config, Key, X>::linearize(const Config& config) const {
	const size_t p = this->p_;

	// extract lagrange multiplier
	Vector lambda = config[this->lagrange_key_];

	// find the error
	Vector g = g_(config);

	// construct the gradient
	Matrix grad = G_(config);

	// construct combined factor
	Matrix Ax = zeros(grad.size1()*2, grad.size2());
	insertSub(Ax, vector_scale(lambda, grad), 0, 0);
	insertSub(Ax, grad, grad.size1(), 0);

	Matrix AL = eye(p*2, p);

	Vector rhs = zero(p*2);
	subInsert(rhs, -1*g, p);

	// construct a mixed constraint model
	Vector sigmas = zero(p*2);
	subInsert(sigmas, ones(p), 0);
	SharedDiagonal mixedConstraint = noiseModel::Constrained::MixedSigmas(sigmas);

	GaussianFactor::shared_ptr factor(new
			GaussianFactor(key_, Ax, this->lagrange_key_, AL, rhs, mixedConstraint));

	return factor;
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
		const LagrangeKey& lagrange_key,
		bool isEquality) :
			NonlinearConstraint<Config>(lagrange_key, dim_constraint, isEquality),
			g_(boost::bind(g, _1)), G1_(boost::bind(G1, _1)), G2_(boost::bind(G2, _1)),
			key1_(key1), key2_(key2)
{
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
		const LagrangeKey& lagrange_key,
		bool isEquality)  :
				NonlinearConstraint<Config>(lagrange_key, dim_constraint, isEquality),
				g_(g), G1_(G1), G2_(G2),
				key1_(key1), key2_(key2)
{
}

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
void NonlinearConstraint2<Config, Key1, X1, Key2, X2>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint2 [" << s << "]: Dim: " << this->p_ << "\n"
			  << "  Key1        : " << (std::string) this->key1_ << "\n"
			  << "  Key2        : " << (std::string) this->key2_ << "\n"
			  << "  Lagrange Key: " << (std::string) this->lagrange_key_ << "\n";
	if (this->isEquality_)
		std::cout << "  Equality Factor" << std::endl;
	else
		std::cout << "  Inequality Factor" << std::endl;
}

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
bool NonlinearConstraint2<Config, Key1, X1, Key2, X2>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint2<Config, Key1, X1, Key2, X2>* p = dynamic_cast<const NonlinearConstraint2<Config, Key1, X1, Key2, X2>*> (&f);
	if (p == NULL) return false;
	if (!(key1_ == p->key1_)) return false;
	if (!(key2_ == p->key2_)) return false;
	if (!(this->lagrange_key_.equals(p->lagrange_key_))) return false;
	if (this->isEquality_ != p->isEquality_) return false;
	return this->p_ == p->p_;
}

/* ************************************************************************* */
template<class Config, class Key1, class X1, class Key2, class X2>
GaussianFactor::shared_ptr
NonlinearConstraint2<Config, Key1, X1, Key2, X2>::linearize(const Config& config) const {
	const size_t p = this->p_;
	// extract lagrange multiplier
	Vector lambda = config[this->lagrange_key_];

	// find the error
	Vector g = g_(config);

	// construct the gradients
	Matrix grad1 = G1_(config);
	Matrix grad2 = G2_(config);

	// create matrices
	Matrix Ax1 = zeros(grad1.size1()*2, grad1.size2()),
		   Ax2 = zeros(grad2.size1()*2, grad2.size2()),
		   AL = eye(p*2, p);

	// insert matrix components
	insertSub(Ax1, vector_scale(lambda, grad1), 0, 0);
	insertSub(Ax1, grad1, grad1.size1(), 0);

	insertSub(Ax2, vector_scale(lambda, grad2), 0, 0);
	insertSub(Ax2, grad2, grad2.size1(), 0);

	Vector rhs = zero(p*2);
	subInsert(rhs, -1*g, p);

	// construct a mixed constraint model
	Vector sigmas = zero(p*2);
	subInsert(sigmas, ones(p), 0);
	SharedDiagonal mixedConstraint = noiseModel::Constrained::MixedSigmas(sigmas);

	GaussianFactor::shared_ptr factor(new
			GaussianFactor(key1_, Ax1, key2_, Ax2, this->lagrange_key_, AL, rhs, mixedConstraint));
	return factor;
}

}
