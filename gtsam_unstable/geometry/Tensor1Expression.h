/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Tensor1Expression.h
 * @brief Tensor expression templates based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * @date Feb 10, 2010
 * @author Frank Dellaert
 */

#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <gtsam_unstable/geometry/tensors.h>

namespace tensors {

	/**
	 * Templated class to provide a rank 1 tensor interface to a class.
	 * This class does not store any data but the result of an expression.
	 * It is associated with an index.
	 * @ingroup tensors
	 * \nosubgrouping
	 */
	template<class A, class I> class Tensor1Expression {

	private:

		A iter;

		typedef Tensor1Expression<A, I> This;

		/** Helper class for multiplying with a double */
		class TimesDouble_ {
			A iter;
			const double s;
		public:
			/// Constructor
			TimesDouble_(const A &a, double s_) :
				iter(a), s(s_) {
			}
			/// Element access
			inline double operator()(int i) const {
				return iter(i) * s;
			}
		};

	public:

		/// @name Standard Constructors
		/// @{

		/** constructor */
		Tensor1Expression(const A &a) :
			iter(a) {
		}

		/// @}
		/// @name Testable
		/// @{

		/** Print */
		void print(const std::string s = "") const {
			std::cout << s << "{";
			std::cout << (*this)(0);
			for (int i = 1; i < I::dim; i++)
				std::cout << ", "<< (*this)(i);
			std::cout << "}" << std::endl;
		}

		/// equality
		template<class B>
		bool equals(const Tensor1Expression<B, I> & q, double tol) const {
			for (int i = 0; i < I::dim; i++)
				if (fabs((*this)(i) - q(i)) > tol) return false;
			return true;
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/** norm */
		double norm() const {
			double sumsqr = 0.0;
			for (int i = 0; i < I::dim; i++)
				sumsqr += iter(i) * iter(i);
			return sqrt(sumsqr);
		}

		/// test equivalence
		template<class B>
		bool equivalent(const Tensor1Expression<B, I> & q, double tol = 1e-9) const {
			return ((*this) * (1.0 / norm())).equals(q * (1.0 / q.norm()), tol)
			|| ((*this) * (-1.0 / norm())).equals(q * (1.0 / q.norm()), tol);
		}

		/** Check if two expressions are equal */
		template<class B>
		bool operator==(const Tensor1Expression<B, I>& e) const {
			for (int i = 0; i < I::dim; i++)
				if (iter(i) != e(i)) return false;
			return true;
		}

		/** element access */
		double operator()(int i) const {
			return iter(i);
		}

		/** mutliply with a double. */
		inline Tensor1Expression<TimesDouble_, I> operator*(double s) const {
			return TimesDouble_(iter, s);
		}

		/** Class for contracting two rank 1 tensor expressions, yielding a double. */
		template<class B>
		inline double operator*(const Tensor1Expression<B, I> &b) const {
			double sum = 0.0;
			for (int i = 0; i < I::dim; i++)
				sum += (*this)(i) * b(i);
			return sum;
		}

	}; // Tensor1Expression

	/// @}
	/// @name Advanced Interface
	/// @{

	/** Print a rank 1 expression */
	template<class A, class I>
	void print(const Tensor1Expression<A, I>& T, const std::string s = "") {
		T.print(s);
	}

	/** norm */
	template<class A, class I>
	double norm(const Tensor1Expression<A, I>& T) {
		return T.norm();
	}

	/**
	 * This template works for any two expressions
	 */
	template<class A, class B, class I>
	bool assert_equality(const Tensor1Expression<A, I>& expected,
			const Tensor1Expression<B, I>& actual, double tol = 1e-9) {
		if (actual.equals(expected, tol)) return true;
		std::cout << "Not equal:\n";
		expected.print("expected:\n");
		actual.print("actual:\n");
		return false;
	}

	/**
	 * This template works for any two expressions
	 */
	template<class A, class B, class I>
	bool assert_equivalent(const Tensor1Expression<A, I>& expected,
			const Tensor1Expression<B, I>& actual, double tol = 1e-9) {
		if (actual.equivalent(expected, tol)) return true;
		std::cout << "Not equal:\n";
		expected.print("expected:\n");
		actual.print("actual:\n");
		return false;
	}

	/// @}

} // namespace tensors
