/**
 * @file    NonlinearFactor.cpp
 * @brief   nonlinear factor versions which assume a Gaussian noise on a measurement 
 * @brief   predicted by a non-linear function h nonlinearFactor
 * @author  Kai Ni
 * @author  Carlos Nieto
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */

#include "NonlinearFactor.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
NonlinearFactor1::NonlinearFactor1(const Vector& z,		                    
                                   const double sigma,	                       
                                   Vector (*h)(const Vector&),                      
                                   const string& key1,                   
                                   Matrix (*H)(const Vector&)) 
  : NonlinearFactor<VectorConfig>(z, sigma), h_(h), key1_(key1), H_(H)
{
  keys_.push_front(key1);
}

/* ************************************************************************* */
void NonlinearFactor1::print(const string& s) const {
	cout << "NonLinearFactor1 " << s << endl;
}

/* ************************************************************************* */
LinearFactor::shared_ptr NonlinearFactor1::linearize(const VectorConfig& c) const {
	// get argument 1 from config
	Vector x1 = c[key1_];

	// Jacobian A = H(x1)/sigma
	Matrix A = H_(x1) / sigma_;

	// Right-hand-side b = error(c) = (z - h(x1))/sigma
	Vector b = (z_ - h_(x1)) / sigma_;

	LinearFactor::shared_ptr p(new LinearFactor(key1_, A, b));
	return p;
}

/* ************************************************************************* */
/** http://artis.imag.fr/~Xavier.Decoret/resources/C++/operator==.html       */
/* ************************************************************************* */
bool NonlinearFactor1::equals(const NonlinearFactor<VectorConfig>& f, double tol) const {
	const NonlinearFactor1* p = dynamic_cast<const NonlinearFactor1*> (&f);
	if (p == NULL) return false;
	return NonlinearFactor<VectorConfig>::equals(*p, tol)
	&& (h_   == p->h_) 
	&& (key1_== p->key1_) 
	&& (H_   == p->H_);
}

/* ************************************************************************* */
NonlinearFactor2::NonlinearFactor2(const Vector& z,
		const double sigma,
		Vector (*h)(const Vector&, const Vector&),
		const string& key1,
		Matrix (*H1)(const Vector&, const Vector&),
		const string& key2,
		Matrix (*H2)(const Vector&, const Vector&)
)
: NonlinearFactor<VectorConfig>(z, sigma), h_(h), key1_(key1), H1_(H1), key2_(key2), H2_(H2)
{
	keys_.push_front(key1);
	keys_.push_front(key2);
}

/* ************************************************************************* */
void NonlinearFactor2::print(const string& s) const {
	cout << "NonLinearFactor2 " << s << endl;
}

/* ************************************************************************* */
LinearFactor::shared_ptr NonlinearFactor2::linearize(const VectorConfig& c) const {
	// get arguments from config
	Vector x1 = c[key1_];
	Vector x2 = c[key2_];

	// Jacobian A = H(x)/sigma
	Matrix A1 = H1_(x1, x2) / sigma_;
	Matrix A2 = H2_(x1, x2) / sigma_;

	// Right-hand-side b = (z - h(x))/sigma
	Vector b = (z_ - h_(x1, x2)) / sigma_;

	LinearFactor::shared_ptr p(new LinearFactor(key1_, A1, key2_, A2, b));
	return p;
}

/* ************************************************************************* */
bool NonlinearFactor2::equals(const NonlinearFactor<VectorConfig>& f, double tol) const {
	const NonlinearFactor2* p = dynamic_cast<const NonlinearFactor2*> (&f);
	if (p == NULL) return false;
	return NonlinearFactor<VectorConfig>::equals(*p, tol)
    && (h_    == p->h_)
    && (key1_ == p->key1_)
    && (H2_   == p->H1_)
    && (key2_ == p->key2_)
    && (H1_   == p->H2_);
}

/* ************************************************************************* */
