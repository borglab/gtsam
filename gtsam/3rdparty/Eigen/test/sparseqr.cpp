// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Desire Nuentsa Wakam <desire.nuentsa_wakam@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
#include "sparse.h"
#include <Eigen/SparseQR>


template<typename MatrixType,typename DenseMat>
int generate_sparse_rectangular_problem(MatrixType& A, DenseMat& dA, int maxRows = 300, int maxCols = 300)
{
  eigen_assert(maxRows >= maxCols);
  typedef typename MatrixType::Scalar Scalar;
  int rows = internal::random<int>(1,maxRows);
  int cols = internal::random<int>(1,rows);
  double density = (std::max)(8./(rows*cols), 0.01);
  
  A.resize(rows,rows);
  dA.resize(rows,rows);
  initSparse<Scalar>(density, dA, A,ForceNonZeroDiag);
  A.makeCompressed();
  int nop = internal::random<int>(0, internal::random<double>(0,1) > 0.5 ? cols/2 : 0);
  for(int k=0; k<nop; ++k)
  {
    int j0 = internal::random<int>(0,cols-1);
    int j1 = internal::random<int>(0,cols-1);
    Scalar s = internal::random<Scalar>();
    A.col(j0)  = s * A.col(j1);
    dA.col(j0) = s * dA.col(j1);
  }
  return rows;
}

template<typename Scalar> void test_sparseqr_scalar()
{
  typedef SparseMatrix<Scalar,ColMajor> MatrixType; 
  typedef Matrix<Scalar,Dynamic,Dynamic> DenseMat;
  typedef Matrix<Scalar,Dynamic,1> DenseVector;
  MatrixType A;
  DenseMat dA;
  DenseVector refX,x,b; 
  SparseQR<MatrixType, AMDOrdering<int> > solver; 
  generate_sparse_rectangular_problem(A,dA);
  
  int n = A.cols();
  b = DenseVector::Random(n);
  solver.compute(A);
  if (solver.info() != Success)
  {
    std::cerr << "sparse QR factorization failed\n";
    exit(0);
    return;
  }
  x = solver.solve(b);
  if (solver.info() != Success)
  {
    std::cerr << "sparse QR factorization failed\n";
    exit(0);
    return;
  } 
  //Compare with a dense QR solver
  ColPivHouseholderQR<DenseMat> dqr(dA);
  refX = dqr.solve(b);
  
  VERIFY_IS_EQUAL(dqr.rank(), solver.rank());
  
  if(solver.rank()<A.cols())
    VERIFY((dA * refX - b).norm() * 2 > (A * x - b).norm() );
  else
    VERIFY_IS_APPROX(x, refX);

  // Compute explicitly the matrix Q
  MatrixType Q, QtQ, idM;
  Q = solver.matrixQ();
  //Check  ||Q' * Q - I ||
  QtQ = Q * Q.adjoint();
  idM.resize(Q.rows(), Q.rows()); idM.setIdentity();
  VERIFY(idM.isApprox(QtQ));
}
void test_sparseqr()
{
  for(int i=0; i<g_repeat; ++i)
  {
    CALL_SUBTEST_1(test_sparseqr_scalar<double>());
    CALL_SUBTEST_2(test_sparseqr_scalar<std::complex<double> >());
  }
}
