/*
 * Tensor5Expression.h
 * @brief Tensor expression templates based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * Created on: Feb 10, 2010
 * @author: Frank Dellaert
 */

#pragma once

#include <iostream>
#include <gtsam/geometry/tensors.h>

namespace tensors {

	/** templated class to interface to an object A as a rank 3 tensor */
	template<class A, class I, class J, class K, class L, class M> class Tensor5Expression {
		A iter;

		typedef Tensor5Expression<A, I, J, K, L, M> This;

		/** Helper class for swapping indices 3 and 4 :-) */
		class Swap34_ {
			const A iter;
		public:
			Swap34_(const A &a) :
				iter(a) {
			}
			double operator()(int i, int j, int k, int l, int m) const {
				return iter(i, j, l, k, m);
			}
		};

	public:

		/** constructor */
		Tensor5Expression(const A &a) :
			iter(a) {
		}

		/** Print */
		void print(const std::string& s = "Tensor5:") const {
			std::cout << s << std::endl;
			for (int m = 0; m < M::dim; m++)
				for (int l = 0; l < L::dim; l++)
					for (int k = 0; k < K::dim; k++) {
						std::cout << "(m,l,k) = (" << m << "," << l << "," << k << ")"
								<< std::endl;
						for (int j = 0; j < J::dim; j++) {
							for (int i = 0; i < I::dim; i++)
								std::cout << " " << (*this)(i, j, k, l, m);
							std::cout << std::endl;
						}
					}
			std::cout << std::endl;
		}

		/** swap indices */
		typedef Tensor5Expression<Swap34_, I, J, L, K, M> Swapped;
		Swapped swap34() {
			return Swap34_(iter);
		}

		/** element access */
		double operator()(int i, int j, int k, int l, int m) const {
			return iter(i, j, k, l, m);
		}

	}; // Tensor5Expression

	/** Print */
	template<class A, class I, class J, class K, class L, class M>
	void print(const Tensor5Expression<A, I, J, K, L, M>& T,
			const std::string& s = "Tensor5:") {
		T.print(s);
	}

	/** Helper class for outer product of rank3 and rank2 tensor */
	template<class A, class I, class J, class K, class B, class L, class M>
	class Rank3Rank2_ {
		typedef Tensor3Expression<A, I, J, K> Rank3;
		typedef Tensor2Expression<B, L, M> Rank2;
		const Rank3 iterA;
		const Rank2 iterB;
	public:
		Rank3Rank2_(const Rank3 &a, const Rank2 &b) :
			iterA(a), iterB(b) {
		}
		double operator()(int i, int j, int k, int l, int m) const {
			return iterA(i, j, k) * iterB(l, m);
		}
	};

	/** outer product of rank2 and rank1 tensor */
	template<class A, class I, class J, class K, class B, class L, class M>
	inline Tensor5Expression<Rank3Rank2_<A, I, J, K, B, L, M> , I, J, K, L, M> operator*(
			const Tensor3Expression<A, I, J, K>& a,
			const Tensor2Expression<B, L, M> &b) {
		return Rank3Rank2_<A, I, J, K, B, L, M> (a, b);
	}

} // namespace tensors
