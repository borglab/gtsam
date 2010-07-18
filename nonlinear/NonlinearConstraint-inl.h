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
NonlinearConstraint<Config>::NonlinearConstraint(size_t dim, double mu) :
	NonlinearFactor<Config>(noiseModel::Constrained::All(dim)),
	mu_(fabs(mu))
{
}

/* ************************************************************************* */
template <class Config>
double NonlinearConstraint<Config>::error(const Config& c) const {
	const Vector error_vector = unwhitenedError(c);
	return mu_ * inner_prod(error_vector, error_vector);
}

/* ************************************************************************* */
// Implementations of unary nonlinear constraints
/* ************************************************************************* */

template <class Config, class Key, class X>
NonlinearConstraint1<Config, Key, X>::NonlinearConstraint1(
			Vector (*g)(const Config& config),
			const Key& key,
			Matrix (*gradG)(const Config& config),
			size_t dim,	double mu) :
				NonlinearConstraint<Config>(dim, mu),
				G_(boost::bind(gradG, _1)), g_(boost::bind(g, _1)), key_(key)
{
	this->keys_.push_back(key);
}

/* ************************************************************************* */
template <class Config, class Key, class X>
NonlinearConstraint1<Config, Key, X>::NonlinearConstraint1(
		boost::function<Vector(const Config& config)> g,
			const Key& key,
			boost::function<Matrix(const Config& config)> gradG,
			size_t dim,	double mu) :
			NonlinearConstraint<Config>(dim, mu),
				G_(gradG), g_(g), key_(key)
{
	this->keys_.push_back(key);
}

/* ************************************************************************* */
template <class Config, class Key, class X>
void NonlinearConstraint1<Config, Key, X>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint1 [" << s << "]: Dim: " << this->dim()
			  << " mu: " << this->mu_ << "\n"
			  << "  Key         : " << (std::string) this->key_ << "\n";
}

/* ************************************************************************* */
template <class Config, class Key, class X>
bool NonlinearConstraint1<Config, Key, X>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint1<Config, Key, X>* p = dynamic_cast<const NonlinearConstraint1<Config, Key, X>*> (&f);
	if (p == NULL) return false;
	if (!(key_ == p->key_)) return false;
	if (fabs(this->mu_ - p->mu_ ) > tol) return false;
	return this->dim() == p->dim();
}

/* ************************************************************************* */
template <class Config, class Key, class X>
GaussianFactor::shared_ptr
NonlinearConstraint1<Config, Key, X>::linearize(const Config& config) const {
	Vector g = -1.0 * g_(config);
	Matrix grad = G_(config);
	SharedDiagonal model = noiseModel::Constrained::All(this->dim());
	GaussianFactor::shared_ptr factor(new GaussianFactor(this->key_, grad, g, model));
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
		size_t dim,	double mu)
	: NonlinearConstraint<Config>(dim, mu),
	  G1_(boost::bind(G1, _1)), G2_(boost::bind(G2, _1)), g_(boost::bind(g, _1)),
	  key1_(key1), key2_(key2)
{
	this->keys_.push_back(key1);
	this->keys_.push_back(key2);
}

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
NonlinearConstraint2<Config, Key1, X1, Key2, X2>::NonlinearConstraint2(
		boost::function<Vector(const Config& config)> g,
		const Key1& key1,
		boost::function<Matrix(const Config& config)> G1,
		const Key2& key2,
		boost::function<Matrix(const Config& config)> G2,
		size_t dim,	double mu)
	: NonlinearConstraint<Config>(dim, mu),
	  G1_(G1), G2_(G2), g_(g),
	  key1_(key1), key2_(key2)
{
	this->keys_.push_back(key1);
	this->keys_.push_back(key2);
}

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
void NonlinearConstraint2<Config, Key1, X1, Key2, X2>::print(const std::string& s) const {
	std::cout << "NonlinearConstraint2 [" << s << "]: Dim: " << this->dim()
			  << " mu: " << this->mu_ << "\n"
			  << "  Key1        : " << (std::string) this->key1_ << "\n"
			  << "  Key2        : " << (std::string) this->key2_ << "\n";
}

/* ************************************************************************* */
template <class Config, class Key1, class X1, class Key2, class X2>
bool NonlinearConstraint2<Config, Key1, X1, Key2, X2>::equals(const Factor<Config>& f, double tol) const {
	const NonlinearConstraint2<Config, Key1, X1, Key2, X2>* p = dynamic_cast<const NonlinearConstraint2<Config, Key1, X1, Key2, X2>*> (&f);
	if (p == NULL) return false;
	if (!(key1_ == p->key1_)) return false;
	if (!(key2_ == p->key2_)) return false;
	if (fabs(this->mu_ - p->mu_ ) > tol) return false;
	return this->dim() == p->dim();
}

/* ************************************************************************* */
template<class Config, class Key1, class X1, class Key2, class X2>
GaussianFactor::shared_ptr
NonlinearConstraint2<Config, Key1, X1, Key2, X2>::linearize(const Config& config) const {
	Vector g = -1.0 * g_(config);
	Matrix grad1 = G1_(config);
	Matrix grad2 = G2_(config);
	SharedDiagonal model = noiseModel::Constrained::All(this->dim());
	GaussianFactor::shared_ptr factor(new GaussianFactor(this->key1_, grad1, this->key2_, grad2, g, model));
	return factor;
}

}
