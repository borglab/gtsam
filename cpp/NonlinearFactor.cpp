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
  : NonlinearFactor<VectorConfig>(z, sigma), h_(h), key_(key1), H_(H)
{
  keys_.push_front(key1);
}

/* ************************************************************************* */
void NonlinearFactor1::print(const string& s) const {
	cout << "NonlinearFactor1 " << s << endl;
  cout << "h  : " << (void*)h_ << endl;
  cout << "key: " << key_      << endl;
  cout << "H  : " << (void*)H_ << endl;
	NonlinearFactor<VectorConfig>::print("parent");
}

/* ************************************************************************* */
GaussianFactor::shared_ptr NonlinearFactor1::linearize(const VectorConfig& c) const {
	// get argument 1 from config
	Vector x1 = c[key_];

	// Jacobian A = H(x1)/sigma
	Matrix A = H_(x1);

	// Right-hand-side b = error(c) = (z - h(x1))/sigma
	Vector b = (z_ - h_(x1));

	GaussianFactor::shared_ptr p(new GaussianFactor(key_, A, b, sigma_));
	return p;
}

/* ************************************************************************* */
/** http://artis.imag.fr/~Xavier.Decoret/resources/C++/operator==.html       */
/* ************************************************************************* */
bool NonlinearFactor1::equals(const Factor<VectorConfig>& f, double tol) const {
	const NonlinearFactor1* p = dynamic_cast<const NonlinearFactor1*> (&f);
	if (p == NULL) return false;
	return NonlinearFactor<VectorConfig>::equals(*p, tol)
	&& (h_   == p->h_) 
	&& (key_ == p->key_)
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
	cout << "NonlinearFactor2 " << s << endl;
  cout << "h   : " << (void*)h_  << endl;
  cout << "key1: " << key1_      << endl;
  cout << "H2  : " << (void*)H2_ << endl;
  cout << "key2: " << key2_      << endl;
  cout << "H1  : " << (void*)H1_ << endl;
	NonlinearFactor<VectorConfig>::print("parent");
}

/* ************************************************************************* */
GaussianFactor::shared_ptr NonlinearFactor2::linearize(const VectorConfig& c) const {
	// get arguments from config
	Vector x1 = c[key1_];
	Vector x2 = c[key2_];

	// Jacobian A = H(x)/sigma
	Matrix A1 = H1_(x1, x2);
	Matrix A2 = H2_(x1, x2);

	// Right-hand-side b = (z - h(x))/sigma
	Vector b = (z_ - h_(x1, x2));

	GaussianFactor::shared_ptr p(new GaussianFactor(key1_, A1, key2_, A2, b, sigma_));
	return p;
}

/* ************************************************************************* */
bool NonlinearFactor2::equals(const Factor<VectorConfig>& f, double tol) const {
	const NonlinearFactor2* p = dynamic_cast<const NonlinearFactor2*> (&f);
	if (p == NULL) return false;
	return NonlinearFactor<VectorConfig>::equals(*p, tol)
    && (h_    == p->h_)
    && (key1_ == p->key1_)
    && (H1_   == p->H1_)
    && (key2_ == p->key2_)
    && (H2_   == p->H2_);
}

/* ************************************************************************* */
