/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file timeMatrix.cpp
 * @brief Performs timing and profiling for Matrix operations
 * @author Alex Cunningham
 */

#include <iostream>
#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>

using namespace std;
using namespace gtsam;

/*
 * Results:
 * Alex's Machine:
 * (using p = 100000 m = 10 n = 12 reps = 50) - Average times
 *  - (1st pass of simple changes) no pass: 0.184  sec , pass: 0.181 sec
 *  - (1st rev memcpy)             no pass: 0.181  sec , pass: 0.180 sec
 *  - (1st rev matrix_range)       no pass: 0.186  sec , pass: 0.184 sec
 * (using p = 10 m = 10 n = 12 reps = 10000000)
 *  - (matrix_range version)       no pass: 24.21  sec , pass: 23.97 sec
 *  - (memcpy version)             no pass: 18.96  sec , pass: 18.39 sec
 *  - (original version)           no pass: 23.45  sec , pass: 22.80 sec
 *  - rev 2100                     no pass: 18.45  sec , pass: 18.35 sec
 */
double timeCollect(size_t p, size_t m, size_t n, bool passDims, size_t reps) {
  // create a large number of matrices
  // p =  number of matrices
  // m =  rows per matrix
  // n =  columns per matrix
  // reps = number of repetitions

  // fill the matrices with identities
  vector<const Matrix *> matrices;
  for (size_t i=0; i<p;++i) {
    Matrix * M = new Matrix;
    (*M) = Matrix::Identity(m,n);
    matrices.push_back(M);
  }

  // start timing
  Matrix result;
  double elapsed;
  {
    gttic_(elapsed);

    if (passDims)
      for (size_t i=0; i<reps; ++i)
        result = collect(matrices, m, n);
    else
      for (size_t i=0; i<reps; ++i)
        result = collect(matrices);

    gttoc_(elapsed);
    tictoc_getNode(elapsedNode, elapsed);
    elapsed = elapsedNode->secs();
    tictoc_reset_();
  }
  // delete the matrices
  for (size_t i=0; i<p;++i) {
    delete matrices[i];
  }

  return elapsed;
  //return elapsed/reps;
}

/*
 * Results:
 * Alex's Machine:
 *  - Original : 0.60 sec (x1000)
 *  - 1st Rev  : 0.49 sec (x1000)
 *  - rev 2100 : 0.52 sec (x1000)
 */
double timeVScaleColumn(size_t m, size_t n, size_t reps) {
  // make a matrix to scale
  Matrix M(m, n);
  for (size_t i=0; i<m; ++i)
    for (size_t j=0; j<n; ++j)
      M(i,j) = 2*i+j;

  // make a vector to use for scaling
  Vector V(m);
  for (size_t i=0; i<m; ++i)
    V(i) = i*2;

  double elapsed;
  Matrix result;
  {
    gttic_(elapsed);

    for (size_t i=0; i<reps; ++i)
      Matrix result = vector_scale(M,V);

    gttoc_(elapsed);
    tictoc_getNode(elapsedNode, elapsed);
    elapsed = elapsedNode->secs();
    tictoc_reset_();
  }

  return elapsed;
}

/*
 * Results:
 * Alex's Machine:
 *  - Original : 0.54 sec (x1000)
 *  - 1st rev  : 0.44 sec (x1000)
 *  - rev 2100 : 1.69 sec (x1000)
 */
double timeVScaleRow(size_t m, size_t n, size_t reps) {
  // make a matrix to scale
  Matrix M(m, n);
  for (size_t i=0; i<m; ++i)
    for (size_t j=0; j<n; ++j)
      M(i,j) = 2*i+j;

  // make a vector to use for scaling
  Vector V(n);
  for (size_t i=0; i<n; ++i)
    V(i) = i*2;

  double elapsed;
  Matrix result;
  {
    gttic_(elapsed);

    for (size_t i=0; i<reps; ++i)
      result = vector_scale(V,M);

    gttoc_(elapsed);
    tictoc_getNode(elapsedNode, elapsed);
    elapsed = elapsedNode->secs();
    tictoc_reset_();
  }

  return elapsed;
}

/**
 * Results:
 * Alex's Machine (reps = 200000)
 *  - ublas matrix_column  : 4.63 sec
 *  - naive implementation : 4.70 sec
 *
 * reps = 2000000
 *  - rev 2100             : 45.11 sec
 */
double timeColumn(size_t reps) {
  // create a matrix
  size_t m = 100; size_t n = 100;
  Matrix M(m, n);
  for (size_t i=0; i<m; ++i)
      for (size_t j=0; j<n; ++j)
        M(i,j) = 2*i+j;

  // extract a column
  double elapsed;
  Vector result;
  {
    gttic_(elapsed);

    for (size_t i=0; i<reps; ++i)
      for (size_t j = 0; j<n; ++j)
        //result = ublas::matrix_column<Matrix>(M, j);
        result = column(M, j);

    gttoc_(elapsed);
    tictoc_getNode(elapsedNode, elapsed);
    elapsed = elapsedNode->secs();
    tictoc_reset_();
  }
  return elapsed;
}

/*
 * Results
 * Alex's machine
 *
 * Runs at reps = 500000
 * Baseline (no householder, just matrix copy) : 0.05 sec
 * Initial                                     : 8.20 sec
 * All in one function                         : 7.89 sec
 * Replace householder update with GSL, ATLAS  : 0.92 sec
 *
 * Runs at reps = 2000000
 * Baseline (GSL/ATLAS householder update)     : 3.61 sec
 *
 * Runs at reps = 5000000
 * Baseline                                    : 8.76 sec
 * GSL/Atlas version of updateAb               : 9.03 sec // Why does this have an effect?
 * Inlining house()                            : 6.33 sec
 * Inlining householder_update [GSL]           : 6.15 sec
 * Rev 2100                                    : 5.75 sec
 *
 */
double timeHouseholder(size_t reps) {
  // create a matrix
  Matrix Abase = (Matrix(4, 7) <<
      -5,  0, 5, 0,  0,  0,  -1,
      00, -5, 0, 5,  0,  0, 1.5,
      10,  0, 0, 0,-10,  0,   2,
      00, 10, 0, 0,  0,-10,  -1).finished();

  // perform timing
  double elapsed;
  {
    gttic_(elapsed);

    for (size_t i=0; i<reps; ++i) {
      Matrix A = Abase;
      householder_(A,3);
    }

    gttoc_(elapsed);
    tictoc_getNode(elapsedNode, elapsed);
    elapsed = elapsedNode->secs();
    tictoc_reset_();
  }
  return elapsed;
}
/**
 * Results: (Alex's machine)
 * reps: 200000
 *
 * Initial (boost matrix proxies) - 12.08
 * Direct pointer method          - 4.62
 */
double timeMatrixInsert(size_t reps) {
  // create a matrix
  Matrix bigBase = Matrix::Zero(100, 100);
  Matrix small = Matrix::Identity(5,5);

  // perform timing
  double elapsed;
  {
    gttic_(elapsed);

    Matrix big = bigBase;
    for (size_t rep=0; rep<reps; ++rep)
      for (size_t i=0; i<100; i += 5)
        for (size_t j=0; j<100; j += 5)
          insertSub(big, small, i,j);

    gttoc_(elapsed);
    tictoc_getNode(elapsedNode, elapsed);
    elapsed = elapsedNode->secs();
    tictoc_reset_();
  }
  return elapsed;
}

int main(int argc, char ** argv) {

  // Time collect()
  cout << "Starting Matrix::collect() Timing" << endl;
  //size_t p = 100000; size_t m = 10; size_t n = 12; size_t reps = 50;
  size_t p = 10; size_t m = 10; size_t n = 12; size_t reps = 10000000;
  double collect_time1 = timeCollect(p, m, n, false, reps);
  double collect_time2 = timeCollect(p, m, n, true, reps);
  cout << "Average Elapsed time for collect (no pass) [" << p << " (" << m << ", " << n << ") matrices] : " << collect_time1 << endl;
  cout << "Average Elapsed time for collect (pass)    [" << p << " (" << m << ", " << n << ") matrices] : " << collect_time2 << endl;

  // Time vector_scale_column
  cout << "Starting Matrix::vector_scale(column) Timing" << endl;
  size_t m1 = 400; size_t n1 = 480; size_t reps1 = 1000;
  double vsColumn_time = timeVScaleColumn(m1, n1, reps1);
  cout << "Elapsed time for vector_scale(column) [(" << m1 << ", " << n1 << ") matrix] : " << vsColumn_time << endl;

  // Time vector_scale_row
  cout << "Starting Matrix::vector_scale(row)    Timing" << endl;
  double vsRow_time = timeVScaleRow(m1, n1, reps1);
  cout << "Elapsed time for vector_scale(row)    [(" << m1 << ", " << n1 << ") matrix] : " << vsRow_time << endl;

  // Time column() NOTE: using the ublas version
  cout << "Starting column() Timing" << endl;
  size_t reps2 = 2000000;
  double column_time = timeColumn(reps2);
  cout << "Time: " << column_time << " sec" << endl;

  // Time householder_ function
  cout << "Starting householder_() Timing" << endl;
  size_t reps_house = 5000000;
  double house_time = timeHouseholder(reps_house);
  cout << "Elapsed time for householder_() : " << house_time << " sec" << endl;

  // Time matrix insertion
  cout << "Starting insertSub() Timing" << endl;
  size_t reps_insert = 200000;
  double insert_time = timeMatrixInsert(reps_insert);
  cout << "Elapsed time for insertSub() : " << insert_time << " sec" << endl;

  return 0;
}
