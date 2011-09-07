/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Tensor3Expression.h
 * @brief Tensor expression templates based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * @date Feb 10, 2010
 * @author Frank Dellaert
 */

#pragma once

#include <iostream>
#include <gtsam/geometry/tensors.h>

namespace tensors {

	/**
	 * templated class to interface to an object A as a rank 3 tensor
	 * @ingroup tensors
	 */
	template<class A, class I, class J, class K> class Tensor3Expression {
		A iter;

		typedef Tensor3Expression<A, I, J, K> This;

		/** Helper class for instantiating one index */
		class FixK_ {
			const int k;
			const A iter;
		public:
			FixK_(int k_, const A &a) :
				k(k_), iter(a) {
			}
			double operator()(int i, int j) const {
				return iter(i, j, k);
			}
		};

		/** Helper class for contracting rank3 and rank1 tensor */
		template<class B> class TimesRank1_ {
			typedef Tensor1Expression<B, I> Rank1;
			const This T;
			const Rank1 t;
		public:
			TimesRank1_(const This &a, const Rank1 &b) :
				T(a), t(b) {
			}
			double operator()(int j, int k) const {
				double sum = 0.0;
				for (int i = 0; i < I::dim; i++)
					sum += T(i, j, k) * t(i);
				return sum;
			}
		};

	public:

		/** constructor */
		Tensor3Expression(const A &a) :
			iter(a) {
		}

		/** Print */
		void print(const std::string& s = "Tensor3:") const {
			std::cout << s << "{";
			(*this)(0).print("");
			for (int k = 1; k < K::dim; k++) {
				std::cout << ",";
				(*this)(k).print("");
			}
			std::cout << "}" << std::endl;
		}

		/// test equality
		template<class B>
		bool equals(const Tensor3Expression<B, I, J, K> & q, double tol) const {
			for (int k = 0; k < K::dim; k++)
				if (!(*this)(k).equals(q(k), tol)) return false;
			return true;
		}

		/** element access */
		double operator()(int i, int j, int k) const {
			return iter(i, j, k);
		}

		/** Fix a single index */
		Tensor2Expression<FixK_, I, J> operator()(int k) const {
			return FixK_(k, iter);
		}

		/** Contracting with rank1 tensor */
		template<class B>
		inline Tensor2Expression<TimesRank1_<B> , J, K> operator*(
				const Tensor1Expression<B, I> &b) const {
			return TimesRank1_<B> (*this, b);
		}

	}; // Tensor3Expression

	/** Print */
	template<class A, class I, class J, class K>
	void print(const Tensor3Expression<A, I, J, K>& T, const std::string& s =
			"Tensor3:") {
		T.print(s);
	}

	/** Helper class for outer product of rank2 and rank1 tensor */
	template<class A, class I, class J, class B, class K>
	class Rank2Rank1_ {
		typedef Tensor2Expression<A, I, J> Rank2;
		typedef Tensor1Expression<B, K> Rank1;
		const Rank2 iterA;
		const Rank1 iterB;
	public:
		/// Constructor
		Rank2Rank1_(const Rank2 &a, const Rank1 &b) :
			iterA(a), iterB(b) {
		}
		/// Element access
		double operator()(int i, int j, int k) const {
			return iterA(i, j) * iterB(k);
		}
	};

	/** outer product of rank2 and rank1 tensor */
	template<class A, class I, class J, class B, class K>
	inline Tensor3Expression<Rank2Rank1_<A, I, J, B, K> , I, J, K> operator*(
			const Tensor2Expression<A, I, J>& a, const Tensor1Expression<B, K> &b) {
		return Rank2Rank1_<A, I, J, B, K> (a, b);
	}

	/** Helper class for outer product of rank1 and rank2 tensor */
	template<class A, class I, class B, class J, class K>
	class Rank1Rank2_ {
		typedef Tensor1Expression<A, I> Rank1;
		typedef Tensor2Expression<B, J, K> Rank2;
		const Rank1 iterA;
		const Rank2 iterB;
	public:
		/// Constructor
		Rank1Rank2_(const Rank1 &a, const Rank2 &b) :
			iterA(a), iterB(b) {
		}
		/// Element access
		double operator()(int i, int j, int k) const {
			return iterA(i) * iterB(j, k);
		}
	};

	/** outer product of rank2 and rank1 tensor */
	template<class A, class I, class J, class B, class K>
	inline Tensor3Expression<Rank1Rank2_<A, I, B, J, K> , I, J, K> operator*(
			const Tensor1Expression<A, I>& a, const Tensor2Expression<B, J, K> &b) {
		return Rank1Rank2_<A, I, B, J, K> (a, b);
	}

	/**
	 * This template works for any two expressions
	 */
	template<class A, class B, class I, class J, class K>
	bool assert_equality(const Tensor3Expression<A, I, J, K>& expected,
			const Tensor3Expression<B, I, J, K>& actual, double tol = 1e-9) {
		if (actual.equals(expected, tol)) return true;
		std::cout << "Not equal:\n";
		expected.print("expected:\n");
		actual.print("actual:\n");
		return false;
	}

} // namespace tensors
