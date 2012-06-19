/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file tensorInterface.h
 * @brief Interfacing tensors template library and gtsam
 * @date Feb 12, 2010
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/tensors.h>
#include <gtsam/base/Matrix.h>

namespace gtsam {

	/** Reshape rank 2 tensor into Matrix */
	template<class A, class I, class J>
	Matrix reshape(const tensors::Tensor2Expression<A, I, J>& T, int m, int n) {
		if (m * n != I::dim * J::dim) throw std::invalid_argument(
				"reshape: incompatible dimensions");
		MatrixRowMajor M(m, n);
		size_t t = 0;
		for (int j = 0; j < J::dim; j++)
			for (int i = 0; i < I::dim; i++)
				M.data()[t++] = T(i, j);
		return Matrix(M);
	}

	/** Reshape rank 2 tensor into Vector */
	template<class A, class I, class J>
	Vector toVector(const tensors::Tensor2Expression<A, I, J>& T) {
		Vector v(I::dim * J::dim);
		size_t t = 0;
		for (int j = 0; j < J::dim; j++)
			for (int i = 0; i < I::dim; i++)
				v(t++) = T(i, j);
		return v;
	}

	/** Reshape Vector into rank 2 tensor */
	template<int N1, int N2>
	tensors::Tensor2<N1, N2> reshape2(const Vector& v) {
		if (v.size() != N1 * N2) throw std::invalid_argument(
				"reshape2: incompatible dimensions");
		double data[N2][N1];
		int t = 0;
		for (int j = 0; j < N2; j++)
			for (int i = 0; i < N1; i++)
				data[j][i] = v(t++);
		return tensors::Tensor2<N1, N2>(data);
	}

  /** Reshape Matrix into rank 2 tensor */
  template<int N1, int N2>
  tensors::Tensor2<N1, N2> reshape2matrix(const Matrix& m) {
    if (m.rows() * m.cols() != N1 * N2) throw std::invalid_argument(
        "reshape2: incompatible dimensions");
    double data[N2][N1];
    for (int j = 0; j < N2; j++)
      for (int i = 0; i < N1; i++)
        data[j][i] = m(j,i);
    return tensors::Tensor2<N1, N2>(data);
  }

	/** Reshape rank 3 tensor into Matrix */
	template<class A, class I, class J, class K>
	Matrix reshape(const tensors::Tensor3Expression<A, I, J, K>& T, int m, int n) {
		if (m * n != I::dim * J::dim * K::dim) throw std::invalid_argument(
				"reshape: incompatible dimensions");
		Matrix M(m, n);
		int t = 0;
		for (int k = 0; k < K::dim; k++)
			for (int i = 0; i < I::dim; i++)
				for (int j = 0; j < J::dim; j++)
					M.data()[t++] = T(i, j, k);
		return M;
	}

	/** Reshape Vector into rank 3 tensor */
	template<int N1, int N2, int N3>
	tensors::Tensor3<N1, N2, N3> reshape3(const Vector& v) {
		if (v.size() != N1 * N2 * N3) throw std::invalid_argument(
				"reshape3: incompatible dimensions");
		double data[N3][N2][N1];
		int t = 0;
		for (int k = 0; k < N3; k++)
			for (int j = 0; j < N2; j++)
				for (int i = 0; i < N1; i++)
					data[k][j][i] = v(t++);
		return tensors::Tensor3<N1, N2, N3>(data);
	}

	/** Reshape rank 5 tensor into Matrix */
	template<class A, class I, class J, class K, class L, class M>
	Matrix reshape(const tensors::Tensor5Expression<A, I, J, K, L, M>& T, int m,
			int n) {
		if (m * n != I::dim * J::dim * K::dim * L::dim * M::dim) throw std::invalid_argument(
				"reshape: incompatible dimensions");
		Matrix R(m, n);
		int t = 0;
		for (int m = 0; m < M::dim; m++)
			for (int l = 0; l < L::dim; l++)
				for (int k = 0; k < K::dim; k++)
					for (int i = 0; i < I::dim; i++)
						for (int j = 0; j < J::dim; j++)
							R.data()[t++] = T(i, j, k, l, m);
		return R;
	}

} // namespace gtsam
