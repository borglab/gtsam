// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2011 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#include "sparse.h"

template<typename SparseMatrixType, typename DenseMatrix, bool IsRowMajor=SparseMatrixType::IsRowMajor> struct test_outer;

template<typename SparseMatrixType, typename DenseMatrix> struct test_outer<SparseMatrixType,DenseMatrix,false> {
  static void run(SparseMatrixType& m2, SparseMatrixType& m4, DenseMatrix& refMat2, DenseMatrix& refMat4) {
    int c  = internal::random(0,m2.cols()-1);
    int c1 = internal::random(0,m2.cols()-1);
    VERIFY_IS_APPROX(m4=m2.col(c)*refMat2.col(c1).transpose(), refMat4=refMat2.col(c)*refMat2.col(c1).transpose());
    VERIFY_IS_APPROX(m4=refMat2.col(c1)*m2.col(c).transpose(), refMat4=refMat2.col(c1)*refMat2.col(c).transpose());
  }
};

template<typename SparseMatrixType, typename DenseMatrix> struct test_outer<SparseMatrixType,DenseMatrix,true> {
  static void run(SparseMatrixType& m2, SparseMatrixType& m4, DenseMatrix& refMat2, DenseMatrix& refMat4) {
    int r  = internal::random(0,m2.rows()-1);
    int c1 = internal::random(0,m2.cols()-1);
    VERIFY_IS_APPROX(m4=m2.row(r).transpose()*refMat2.col(c1).transpose(), refMat4=refMat2.row(r).transpose()*refMat2.col(c1).transpose());
    VERIFY_IS_APPROX(m4=refMat2.col(c1)*m2.row(r), refMat4=refMat2.col(c1)*refMat2.row(r));
  }
};

// (m2,m4,refMat2,refMat4,dv1);
//     VERIFY_IS_APPROX(m4=m2.innerVector(c)*dv1.transpose(), refMat4=refMat2.colVector(c)*dv1.transpose());
//     VERIFY_IS_APPROX(m4=dv1*mcm.col(c).transpose(), refMat4=dv1*refMat2.col(c).transpose());

template<typename SparseMatrixType> void sparse_product()
{
  typedef typename SparseMatrixType::Index Index;
  Index n = 100;
  const Index rows  = internal::random<int>(1,n);
  const Index cols  = internal::random<int>(1,n);
  const Index depth = internal::random<int>(1,n);
  typedef typename SparseMatrixType::Scalar Scalar;
  enum { Flags = SparseMatrixType::Flags };

  double density = (std::max)(8./(rows*cols), 0.01);
  typedef Matrix<Scalar,Dynamic,Dynamic> DenseMatrix;
  typedef Matrix<Scalar,Dynamic,1> DenseVector;

  Scalar s1 = internal::random<Scalar>();
  Scalar s2 = internal::random<Scalar>();

  // test matrix-matrix product
  {
    DenseMatrix refMat2  = DenseMatrix::Zero(rows, depth);
    DenseMatrix refMat2t = DenseMatrix::Zero(depth, rows);
    DenseMatrix refMat3  = DenseMatrix::Zero(depth, cols);
    DenseMatrix refMat3t = DenseMatrix::Zero(cols, depth);
    DenseMatrix refMat4  = DenseMatrix::Zero(rows, cols);
    DenseMatrix refMat4t = DenseMatrix::Zero(cols, rows);
    DenseMatrix refMat5  = DenseMatrix::Random(depth, cols);
    DenseMatrix refMat6  = DenseMatrix::Random(rows, rows);
    DenseMatrix dm4 = DenseMatrix::Zero(rows, rows);
//     DenseVector dv1 = DenseVector::Random(rows);
    SparseMatrixType m2 (rows, depth);
    SparseMatrixType m2t(depth, rows);
    SparseMatrixType m3 (depth, cols);
    SparseMatrixType m3t(cols, depth);
    SparseMatrixType m4 (rows, cols);
    SparseMatrixType m4t(cols, rows);
    SparseMatrixType m6(rows, rows);
    initSparse(density, refMat2,  m2);
    initSparse(density, refMat2t, m2t);
    initSparse(density, refMat3,  m3);
    initSparse(density, refMat3t, m3t);
    initSparse(density, refMat4,  m4);
    initSparse(density, refMat4t, m4t);
    initSparse(density, refMat6, m6);

//     int c = internal::random<int>(0,depth-1);

    // sparse * sparse
    VERIFY_IS_APPROX(m4=m2*m3, refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(m4=m2t.transpose()*m3, refMat4=refMat2t.transpose()*refMat3);
    VERIFY_IS_APPROX(m4=m2t.transpose()*m3t.transpose(), refMat4=refMat2t.transpose()*refMat3t.transpose());
    VERIFY_IS_APPROX(m4=m2*m3t.transpose(), refMat4=refMat2*refMat3t.transpose());

    VERIFY_IS_APPROX(m4 = m2*m3/s1, refMat4 = refMat2*refMat3/s1);
    VERIFY_IS_APPROX(m4 = m2*m3*s1, refMat4 = refMat2*refMat3*s1);
    VERIFY_IS_APPROX(m4 = s2*m2*m3*s1, refMat4 = s2*refMat2*refMat3*s1);

    VERIFY_IS_APPROX(m4=(m2*m3).pruned(0), refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(m4=(m2t.transpose()*m3).pruned(0), refMat4=refMat2t.transpose()*refMat3);
    VERIFY_IS_APPROX(m4=(m2t.transpose()*m3t.transpose()).pruned(0), refMat4=refMat2t.transpose()*refMat3t.transpose());
    VERIFY_IS_APPROX(m4=(m2*m3t.transpose()).pruned(0), refMat4=refMat2*refMat3t.transpose());

    // test aliasing
    m4 = m2; refMat4 = refMat2;
    VERIFY_IS_APPROX(m4=m4*m3, refMat4=refMat4*refMat3);

    // sparse * dense
    VERIFY_IS_APPROX(dm4=m2*refMat3, refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4=m2*refMat3t.transpose(), refMat4=refMat2*refMat3t.transpose());
    VERIFY_IS_APPROX(dm4=m2t.transpose()*refMat3, refMat4=refMat2t.transpose()*refMat3);
    VERIFY_IS_APPROX(dm4=m2t.transpose()*refMat3t.transpose(), refMat4=refMat2t.transpose()*refMat3t.transpose());

    VERIFY_IS_APPROX(dm4=m2*(refMat3+refMat3), refMat4=refMat2*(refMat3+refMat3));
    VERIFY_IS_APPROX(dm4=m2t.transpose()*(refMat3+refMat5)*0.5, refMat4=refMat2t.transpose()*(refMat3+refMat5)*0.5);

    // dense * sparse
    VERIFY_IS_APPROX(dm4=refMat2*m3, refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4=refMat2*m3t.transpose(), refMat4=refMat2*refMat3t.transpose());
    VERIFY_IS_APPROX(dm4=refMat2t.transpose()*m3, refMat4=refMat2t.transpose()*refMat3);
    VERIFY_IS_APPROX(dm4=refMat2t.transpose()*m3t.transpose(), refMat4=refMat2t.transpose()*refMat3t.transpose());

    // sparse * dense and dense * sparse outer product
    test_outer<SparseMatrixType,DenseMatrix>::run(m2,m4,refMat2,refMat4);

    VERIFY_IS_APPROX(m6=m6*m6, refMat6=refMat6*refMat6);
  }

  // test matrix - diagonal product
  {
    DenseMatrix refM2 = DenseMatrix::Zero(rows, rows);
    DenseMatrix refM3 = DenseMatrix::Zero(rows, rows);
    DiagonalMatrix<Scalar,Dynamic> d1(DenseVector::Random(rows));
    SparseMatrixType m2(rows, rows);
    SparseMatrixType m3(rows, rows);
    initSparse<Scalar>(density, refM2, m2);
    initSparse<Scalar>(density, refM3, m3);
    VERIFY_IS_APPROX(m3=m2*d1, refM3=refM2*d1);
    VERIFY_IS_APPROX(m3=m2.transpose()*d1, refM3=refM2.transpose()*d1);
    VERIFY_IS_APPROX(m3=d1*m2, refM3=d1*refM2);
    VERIFY_IS_APPROX(m3=d1*m2.transpose(), refM3=d1 * refM2.transpose());
  }

  // test self adjoint products
  {
    DenseMatrix b = DenseMatrix::Random(rows, rows);
    DenseMatrix x = DenseMatrix::Random(rows, rows);
    DenseMatrix refX = DenseMatrix::Random(rows, rows);
    DenseMatrix refUp = DenseMatrix::Zero(rows, rows);
    DenseMatrix refLo = DenseMatrix::Zero(rows, rows);
    DenseMatrix refS = DenseMatrix::Zero(rows, rows);
    SparseMatrixType mUp(rows, rows);
    SparseMatrixType mLo(rows, rows);
    SparseMatrixType mS(rows, rows);
    do {
      initSparse<Scalar>(density, refUp, mUp, ForceRealDiag|/*ForceNonZeroDiag|*/MakeUpperTriangular);
    } while (refUp.isZero());
    refLo = refUp.adjoint();
    mLo = mUp.adjoint();
    refS = refUp + refLo;
    refS.diagonal() *= 0.5;
    mS = mUp + mLo;
    // TODO be able to address the diagonal....
    for (int k=0; k<mS.outerSize(); ++k)
      for (typename SparseMatrixType::InnerIterator it(mS,k); it; ++it)
        if (it.index() == k)
          it.valueRef() *= 0.5;

    VERIFY_IS_APPROX(refS.adjoint(), refS);
    VERIFY_IS_APPROX(mS.adjoint(), mS);
    VERIFY_IS_APPROX(mS, refS);
    VERIFY_IS_APPROX(x=mS*b, refX=refS*b);

    VERIFY_IS_APPROX(x=mUp.template selfadjointView<Upper>()*b, refX=refS*b);
    VERIFY_IS_APPROX(x=mLo.template selfadjointView<Lower>()*b, refX=refS*b);
    VERIFY_IS_APPROX(x=mS.template selfadjointView<Upper|Lower>()*b, refX=refS*b);
  }
}

// New test for Bug in SparseTimeDenseProduct
template<typename SparseMatrixType, typename DenseMatrixType> void sparse_product_regression_test()
{
  // This code does not compile with afflicted versions of the bug
  SparseMatrixType sm1(3,2);
  DenseMatrixType m2(2,2);
  sm1.setZero();
  m2.setZero();

  DenseMatrixType m3 = sm1*m2;


  // This code produces a segfault with afflicted versions of another SparseTimeDenseProduct
  // bug

  SparseMatrixType sm2(20000,2);
  sm2.setZero();
  DenseMatrixType m4(sm2*m2);

  VERIFY_IS_APPROX( m4(0,0), 0.0 );
}

void test_sparse_product()
{
  for(int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1( (sparse_product<SparseMatrix<double,ColMajor> >()) );
    CALL_SUBTEST_1( (sparse_product<SparseMatrix<double,RowMajor> >()) );
    CALL_SUBTEST_2( (sparse_product<SparseMatrix<std::complex<double>, ColMajor > >()) );
    CALL_SUBTEST_2( (sparse_product<SparseMatrix<std::complex<double>, RowMajor > >()) );
    CALL_SUBTEST_4( (sparse_product_regression_test<SparseMatrix<double,RowMajor>, Matrix<double, Dynamic, Dynamic, RowMajor> >()) );
  }
}
