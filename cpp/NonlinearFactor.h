/**
 * @file    NonlinearFactor.h
 * @brief   Non-linear factor class
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

// \callgraph

#pragma once

#include <list>
#include <limits>

#include <boost/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>

#include "Factor.h"
#include "Vector.h"
#include "Matrix.h"
#include "NoiseModel.h"
#include "GaussianFactor.h"

#define INSTANTIATE_NONLINEAR_FACTOR1(C,J,X) \
  template class gtsam::NonlinearFactor1<C,J,X>;
#define INSTANTIATE_NONLINEAR_FACTOR2(C,J1,X1,J2,X2) \
    template class gtsam::NonlinearFactor2<C,J1,X1,J2,X2>;

namespace gtsam {

	/**
	 * Nonlinear factor which assumes zero-mean Gaussian noise on the
	 * on a measurement predicted by a non-linear function h.
	 *
	 * Templated on a configuration type. The configurations are typically
	 * more general than just vectors, e.g., Rot3 or Pose3,
	 * which are objects in non-linear manifolds (Lie groups).
	 */
	template<class Config>
	class NonlinearFactor: public Factor<Config> {

	protected:

		typedef NonlinearFactor<Config> This;

		sharedGaussian noiseModel_; /** Noise model */
		std::list<Symbol> keys_; /** cached keys */

	public:

		/** Default constructor for I/O only */
		NonlinearFactor() {
		}

		/**
		 *  Constructor
		 *  @param noiseModel shared pointer to a noise model
		 */
		NonlinearFactor(const sharedGaussian& noiseModel) :
			noiseModel_(noiseModel) {
		}

		/** print */
		void print(const std::string& s = "") const {
			std::cout << "NonlinearFactor " << s << std::endl;
			noiseModel_->print("noise model");
		}

		/** Check if two NonlinearFactor objects are equal */
		bool equals(const Factor<Config>& f, double tol = 1e-9) const {
			const This* p = dynamic_cast<const NonlinearFactor<Config>*> (&f);
			if (p == NULL) return false;
			return noiseModel_->equals(*p->noiseModel_, tol);
		}

		/**
		 * calculate the error of the factor
		 */
		double error(const Config& c) const {
			return 0.5 * noiseModel_->Mahalanobis(unwhitenedError(c));
		}

		/** return keys */
		std::list<Symbol> keys() const {
			return keys_;
		}

		/** get the size of the factor */
		std::size_t size() const {
			return keys_.size();
		}

		/** Vector of errors, unwhitened ! */
		virtual Vector unwhitenedError(const Config& c) const = 0;

		/** Vector of errors, whitened ! */
		Vector whitenedError(const Config& c) const {
			return noiseModel_->whiten(unwhitenedError(c));
		}

		/** linearize to a GaussianFactor */
		virtual boost::shared_ptr<GaussianFactor>
		linearize(const Config& c) const = 0;

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			// TODO NoiseModel
		}

	}; // NonlinearFactor


	/**
	 * A Gaussian nonlinear factor that takes 1 parameter
	 * implementing the density P(z|x) \propto exp -0.5*|z-h(x)|^2_C
	 * Templated on the parameter type X and the configuration Config
	 * There is no return type specified for h(x). Instead, we require
	 * the derived class implements error_vector(c) = h(x)-z \approx Ax-b
	 * This allows a graph to have factors with measurements of mixed type.
	 */
	template<class Config, class Key, class X>
	class NonlinearFactor1: public NonlinearFactor<Config> {

	protected:

		// The value of the key. Not const to allow serialization
		Key key_;

		typedef NonlinearFactor<Config> Base;
		typedef NonlinearFactor1<Config, Key, X> This;

	public:

		/** Default constructor for I/O only */
		NonlinearFactor1() {
		}

		/**
		 *  Constructor
		 *  @param z measurement
		 *  @param key by which to look up X value in Config
		 */
		NonlinearFactor1(const sharedGaussian& noiseModel,
				const Key& key1) :
			Base(noiseModel), key_(key1) {
			this->keys_.push_back(key_);
		}

		/* print */
		void print(const std::string& s = "") const {
			std::cout << "NonlinearFactor1 " << s << std::endl;
			std::cout << "key: " << (std::string) key_ << std::endl;
			Base::print("parent");
		}

		/** Check if two factors are equal. Note type is Factor and needs cast. */
		bool equals(const Factor<Config>& f, double tol = 1e-9) const {
			const This* p = dynamic_cast<const This*> (&f);
			if (p == NULL) return false;
			return Base::equals(*p, tol) && (key_ == p->key_);
		}

		/** error function h(x)-z, unwhitened !!! */
		inline Vector unwhitenedError(const Config& x) const {
			const Key& j = key_;
			const X& xj = x[j];
			return evaluateError(xj);
		}

		/**
		 * Linearize a non-linearFactor1 to get a GaussianFactor
		 * Ax-b \approx h(x0+dx)-z = h(x0) + A*dx - z
		 * Hence b = z - h(x0) = - error_vector(x)
		 */
		virtual boost::shared_ptr<GaussianFactor> linearize(const Config& x) const {
			const X& xj = x[key_];
			Matrix A;
			Vector b = - evaluateError(xj, A);
			// TODO pass unwhitened + noise model to Gaussian factor
			this->noiseModel_->WhitenInPlace(A);
			this->noiseModel_->whitenInPlace(b);
			return GaussianFactor::shared_ptr(new GaussianFactor(key_, A, b, 1.0));
		}

		/*
		 *  Override this method to finish implementing a unary factor.
		 *  If the optional Matrix reference argument is specified, it should compute
		 *  both the function evaluation and its derivative in X.
		 */
		virtual Vector evaluateError(const X& x, boost::optional<Matrix&> H =
				boost::none) const = 0;

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor",
					boost::serialization::base_object<NonlinearFactor>(*this));
			ar & BOOST_SERIALIZATION_NVP(key_);
		}

	};

	/**
	 * A Gaussian nonlinear factor that takes 2 parameters
	 * Note: cannot be serialized as contains function pointers
	 * Specialized derived classes could do this
	 */
	template<class Config, class Key1, class X1, class Key2, class X2>
	class NonlinearFactor2: public NonlinearFactor<Config> {

	protected:

		// The values of the keys. Not const to allow serialization
		Key1 key1_;
		Key2 key2_;

		typedef NonlinearFactor<Config> Base;
		typedef NonlinearFactor2<Config, Key1, X1, Key2, X2> This;

	public:

		/**
		 * Default Constructor for I/O
		 */
		NonlinearFactor2() {
		}

		/**
		 * Constructor
		 * @param j1 key of the first variable
		 * @param j2 key of the second variable
		 */
		NonlinearFactor2(const sharedGaussian& noiseModel, Key1 j1,
				Key2 j2) :
			Base(noiseModel), key1_(j1), key2_(j2) {
			this->keys_.push_back(key1_);
			this->keys_.push_back(key2_);
		}

		/** Print */
		void print(const std::string& s = "") const {
			std::cout << "NonlinearFactor2 " << s << std::endl;
			std::cout << "key1: " << (std::string) key1_ << std::endl;
			std::cout << "key2: " << (std::string) key2_ << std::endl;
			Base::print("parent");
		}

		/** Check if two factors are equal */
		bool equals(const Factor<Config>& f, double tol = 1e-9) const {
			const This* p = dynamic_cast<const This*> (&f);
			if (p == NULL) return false;
			return Base::equals(*p, tol) && (key1_ == p->key1_)
					&& (key2_ == p->key2_);
		}

		/** error function z-h(x1,x2) */
		inline Vector unwhitenedError(const Config& x) const {
			const X1& x1 = x[key1_];
			const X2& x2 = x[key2_];
			return evaluateError(x1, x2);
		}

		/**
		 * Linearize a non-linearFactor1 to get a GaussianFactor
		 * Ax-b \approx h(x1+dx1,x2+dx2)-z = h(x1,x2) + A2*dx1 + A2*dx2 - z
		 * Hence b = z - h(x1,x2) = - error_vector(x)
		 */
		boost::shared_ptr<GaussianFactor> linearize(const Config& c) const {
			const X1& x1 = c[key1_];
			const X2& x2 = c[key2_];
			Matrix A1, A2;
			Vector b = -evaluateError(x1, x2, A1, A2);
			// TODO pass unwhitened + noise model to Gaussian factor
			this->noiseModel_->WhitenInPlace(A1);
			this->noiseModel_->WhitenInPlace(A2);
			this->noiseModel_->whitenInPlace(b);
			return GaussianFactor::shared_ptr(new GaussianFactor(key1_, A1, key2_,
					A2, b, 1.0));
		}

		/** methods to retrieve both keys */
		inline const Key1& key1() const {
			return key1_;
		}
		inline const Key2& key2() const {
			return key2_;
		}

		/*
		 *  Override this method to finish implementing a binary factor.
		 *  If any of the optional Matrix reference arguments are specified, it should compute
		 *  both the function evaluation and its derivative(s) in X1 (and/or X2).
		 */
		virtual Vector
		evaluateError(const X1&, const X2&, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none) const = 0;

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor",
					boost::serialization::base_object<NonlinearFactor>(*this));
			ar & BOOST_SERIALIZATION_NVP(key1_);
			ar & BOOST_SERIALIZATION_NVP(key2_);
		}

	};

/* ************************************************************************* */
}
