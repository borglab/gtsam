/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Tensor2Expression.h
 * @brief Tensor expression templates based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * @date Feb 10, 2010
 * @author Frank Dellaert
 */

#pragma once

#include <stdexcept>
#include <iostream>
#include <gtsam/geometry/tensors.h>

namespace tensors {

	/**
	 * Templated class to hold a rank 2 tensor expression.
	 * @addtogroup tensors
	 * \nosubgrouping
	 */
	template<class A, class I, class J> class Tensor2Expression {

	private:

		A iter;

		typedef Tensor2Expression<A, I, J> This;

		/** Helper class for instantiating one index */
		class FixJ_ {
			const int j;
			const A iter;
		public:
			FixJ_(int j_, const A &a) :
					j(j_), iter(a) {
			}
			double operator()(int i) const {
				return iter(i, j);
			}
		};

		/** Helper class for swapping indices */
		class Swap_ {
			const A iter;
		public:
			/// Constructor
			Swap_(const A &a) :
					iter(a) {
			}
			/// Element access
			double operator()(int j, int i) const {
				return iter(i, j);
			}
		};

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
			inline double operator()(int i, int j) const {
				return iter(i, j) * s;
			}
		};

		/** Helper class for contracting index I with rank 1 tensor */
		template<class B> class ITimesRank1_ {
			const This a;
			const Tensor1Expression<B, I> b;
		public:
			/// Constructor
			ITimesRank1_(const This &a_, const Tensor1Expression<B, I> &b_) :
					a(a_), b(b_) {
			}
			/// Element access
			double operator()(int j) const {
				double sum = 0.0;
				for (int i = 0; i < I::dim; i++)
					sum += a(i, j) * b(i);
				return sum;
			}
		};

		/** Helper class for contracting index J with rank 1 tensor */
		template<class B> class JTimesRank1_ {
			const This a;
			const Tensor1Expression<B, J> b;
		public:
			/// Constructor
			JTimesRank1_(const This &a_, const Tensor1Expression<B, J> &b_) :
					a(a_), b(b_) {
			}
			/// Element access
			double operator()(int i) const {
				double sum = 0.0;
				for (int j = 0; j < J::dim; j++)
					sum += a(i, j) * b(j);
				return sum;
			}
		};

		/** Helper class for contracting index I with rank 2 tensor */
		template<class B, class K> class ITimesRank2_ {
			const This a;
			const Tensor2Expression<B, I, K> b;
		public:
			/// Constructor
			ITimesRank2_(const This &a_, const Tensor2Expression<B, I, K> &b_) :
					a(a_), b(b_) {
			}
			/// Element access
			double operator()(int j, int k) const {
				double sum = 0.0;
				for (int i = 0; i < I::dim; i++)
					sum += a(i, j) * b(i, k);
				return sum;
			}
		};

	public:

		/// @name Standard Constructors
		/// @{

		/** constructor */
		Tensor2Expression(const A &a) :
				iter(a) {
		}

		/// @}
		/// @name Testable
		/// @{

		/** Print */
		void print(const std::string& s = "Tensor2:") const {
			std::cout << s << "{";
			(*this)(0).print();
			for (int j = 1; j < J::dim; j++) {
				std::cout << ",";
				(*this)(j).print("");
			}
			std::cout << "}" << std::endl;
		}

		/// test equality
		template<class B>
		bool equals(const Tensor2Expression<B, I, J> & q, double tol) const {
			for (int j = 0; j < J::dim; j++)
				if (!(*this)(j).equals(q(j), tol))
					return false;
			return true;
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/** norm */
		double norm() const {
			double sumsqr = 0.0;
			for (int i = 0; i < I::dim; i++)
				for (int j = 0; j < J::dim; j++)
					sumsqr += iter(i, j) * iter(i, j);
			return sqrt(sumsqr);
		}

		/// test equivalence
		template<class B>
		bool equivalent(const Tensor2Expression<B, I, J> & q, double tol) const {
			return ((*this) * (1.0 / norm())).equals(q * (1.0 / q.norm()), tol)
					|| ((*this) * (-1.0 / norm())).equals(q * (1.0 / q.norm()), tol);
		}

		/** element access */
		double operator()(int i, int j) const {
			return iter(i, j);
		}

		/** swap indices */
		typedef Tensor2Expression<Swap_, J, I> Swapped;
		/// Return Swap_ helper class
		Swapped swap() {
			return Swap_(iter);
		}

		/** mutliply with a double. */
		inline Tensor2Expression<TimesDouble_, I, J> operator*(double s) const {
			return TimesDouble_(iter, s);
		}

		/** Fix a single index */
		Tensor1Expression<FixJ_, I> operator()(int j) const {
			return FixJ_(j, iter);
		}

		/** Check if two expressions are equal */
		template<class B>
		bool operator==(const Tensor2Expression<B, I, J>& T) const {
			for (int i = 0; i < I::dim; i++)
				for (int j = 0; j < J::dim; j++)
					if (iter(i, j) != T(i, j))
						return false;
			return true;
		}

		/// @}
		/// @name Advanced Interface
		/// @{

		/** c(j) = a(i,j)*b(i) */
		template<class B>
		inline Tensor1Expression<ITimesRank1_<B>, J> operator*(
				const Tensor1Expression<B, I>& p) {
			return ITimesRank1_<B>(*this, p);
		}

		/** c(i) = a(i,j)*b(j) */
		template<class B>
		inline Tensor1Expression<JTimesRank1_<B>, I> operator*(
				const Tensor1Expression<B, J> &p) {
			return JTimesRank1_<B>(*this, p);
		}

		/** c(j,k) = a(i,j)*T(i,k) */
		template<class B, class K>
		inline Tensor2Expression<ITimesRank2_<B, K> , J, K> operator*(
				const Tensor2Expression<B, I, K>& p) {
			return ITimesRank2_<B, K>(*this, p);
		}

	};
	// Tensor2Expression

	/** Print */
	template<class A, class I, class J>
	void print(const Tensor2Expression<A, I, J>& T, const std::string& s =
			"Tensor2:") {
		T.print(s);
	}

	/** Helper class for multiplying two covariant tensors */
	template<class A, class I, class B, class J> class Rank1Rank1_ {
		const Tensor1Expression<A, I> iterA;
		const Tensor1Expression<B, J> iterB;
	public:
		/// Constructor
		Rank1Rank1_(const Tensor1Expression<A, I> &a,
		const Tensor1Expression<B, J> &b) :
				iterA(a), iterB(b) {
		}
		/// element access
		double operator()(int i, int j) const {
			return iterA(i) * iterB(j);
		}
	};

	/** Multiplying two different indices yields an outer product */
	template<class A, class I, class B, class J>
	inline Tensor2Expression<Rank1Rank1_<A, I, B, J> , I, J> operator*(
			const Tensor1Expression<A, I> &a, const Tensor1Expression<B, J> &b) {
		return Rank1Rank1_<A, I, B, J>(a, b);
	}

	/**
	 * This template works for any two expressions
	 */
	template<class A, class B, class I, class J>
	bool assert_equality(const Tensor2Expression<A, I, J>& expected,
	const Tensor2Expression<B, I, J>& actual, double tol = 1e-9) {
		if (actual.equals(expected, tol))
			return true;
		std::cout << "Not equal:\n";
		expected.print("expected:\n");
		actual.print("actual:\n");
		return false;
	}

	/**
	 * This template works for any two expressions
	 */
	template<class A, class B, class I, class J>
	bool assert_equivalent(const Tensor2Expression<A, I, J>& expected,
	const Tensor2Expression<B, I, J>& actual, double tol = 1e-9) {
		if (actual.equivalent(expected, tol))
			return true;
		std::cout << "Not equal:\n";
		expected.print("expected:\n");
		actual.print("actual:\n");
		return false;
	}

	/// @}

} // namespace tensors
