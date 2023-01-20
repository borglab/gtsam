/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeublas.cpp
 * @brief   Tests to help determine which way of accomplishing something with Eigen is faster
 * @author  Richard Roberts
 * @date    Sep 18, 2010
 */

#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>

#include <iostream>
#include <random>
#include <vector>
#include <utility>

using namespace std;
//namespace ublas = boost::numeric::ublas;
//using namespace Eigen;

static std::mt19937 rng;
static std::uniform_real_distribution<> uniform(-1.0, 0.0);
//typedef ublas::matrix<double> matrix;
//typedef ublas::matrix_range<matrix> matrix_range;
//typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix;
//typedef Eigen::Block<matrix> matrix_block;

//using ublas::range;
//using ublas::triangular_matrix;

int main(int argc, char* argv[]) {

  if(true) {
    cout << "\nTiming matrix_block:" << endl;

    // We use volatile here to make these appear to the optimizing compiler as
    // if their values are only known at run-time.
    volatile size_t m=500;
    volatile size_t n=300;
    volatile size_t nReps = 1000;
    assert(m > n);
    std::uniform_int_distribution<size_t> uniform_i(0,m-1);
    std::uniform_int_distribution<size_t> uniform_j(0,n-1);
    gtsam::Matrix mat((int)m,(int)n);
    gtsam::SubMatrix full = mat.block(0, 0, m, n);
    gtsam::SubMatrix top = mat.block(0, 0, n, n);
    gtsam::SubMatrix block = mat.block(m/4, n/4, m-m/2, n-n/2);

    cout << "  Basic: " << (int)m << "x" << (int)n << endl;
    cout << "  Full:  mat(" << 0 << ":" << (int)m << ", " << 0 << ":" << (int)n << ")" << endl;
    cout << "  Top:   mat(" << 0 << ":" << (int)n << ", " << 0 << ":" << (int)n << ")" << endl;
    cout << "  Block: mat(" << size_t(m/4) << ":" << size_t(m-m/4) << ", " << size_t(n/4) << ":" << size_t(n-n/4) << ")" << endl;
    cout << endl;

    {
      double basicTime, fullTime, topTime, blockTime;

      cout << "Row-major matrix, row-major assignment:" << endl;

      // Do a few initial assignments to let any cache effects stabilize
      for(size_t rep=0; rep<1000; ++rep)
        for(size_t i=0; i<(size_t)mat.rows(); ++i)
          for(size_t j=0; j<(size_t)mat.cols(); ++j)
            mat(i,j) = uniform(rng);

      gttic_(basicTime);
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t i=0; i<(size_t)mat.rows(); ++i)
          for(size_t j=0; j<(size_t)mat.cols(); ++j)
            mat(i,j) = uniform(rng);
      gttoc_(basicTime);
      tictoc_getNode(basicTimeNode, basicTime);
      basicTime = basicTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Basic: " << double(1000000 * basicTime / double(mat.rows()*mat.cols()*nReps)) << " mus/element" << endl;

      gttic_(fullTime);
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t i=0; i<(size_t)full.rows(); ++i)
          for(size_t j=0; j<(size_t)full.cols(); ++j)
            full(i,j) = uniform(rng);
      gttoc_(fullTime);
      tictoc_getNode(fullTimeNode, fullTime);
      fullTime = fullTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Full:  " << double(1000000 * fullTime / double(full.rows()*full.cols()*nReps)) << " mus/element" << endl;

      gttic_(topTime);
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t i=0; i<(size_t)top.rows(); ++i)
          for(size_t j=0; j<(size_t)top.cols(); ++j)
            top(i,j) = uniform(rng);
      gttoc_(topTime);
      tictoc_getNode(topTimeNode, topTime);
      topTime = topTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Top:   " << double(1000000 * topTime / double(top.rows()*top.cols()*nReps)) << " mus/element" << endl;

      gttic_(blockTime);
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t i=0; i<(size_t)block.rows(); ++i)
          for(size_t j=0; j<(size_t)block.cols(); ++j)
            block(i,j) = uniform(rng);
      gttoc_(blockTime);
      tictoc_getNode(blockTimeNode, blockTime);
      blockTime = blockTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Block: " << double(1000000 * blockTime / double(block.rows()*block.cols()*nReps)) << " mus/element" << endl;

      cout << endl;
    }

    {
      double basicTime, fullTime, topTime, blockTime;

      cout << "Row-major matrix, column-major assignment:" << endl;

      // Do a few initial assignments to let any cache effects stabilize
      for(size_t rep=0; rep<1000; ++rep)
        for(size_t j=0; j<(size_t)mat.cols(); ++j)
          for(size_t i=0; i<(size_t)mat.rows(); ++i)
            mat(i,j) = uniform(rng);

      gttic_(basicTime);
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t j=0; j<(size_t)mat.cols(); ++j)
          for(size_t i=0; i<(size_t)mat.rows(); ++i)
            mat(i,j) = uniform(rng);
      gttoc_(basicTime);
      tictoc_getNode(basicTimeNode, basicTime);
      basicTime = basicTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Basic: " << double(1000000 * basicTime / double(mat.rows()*mat.cols()*nReps)) << " mus/element" << endl;

      gttic_(fullTime);
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t j=0; j<(size_t)full.cols(); ++j)
          for(size_t i=0; i<(size_t)full.rows(); ++i)
            full(i,j) = uniform(rng);
      gttoc_(fullTime);
      tictoc_getNode(fullTimeNode, fullTime);
      fullTime = fullTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Full:  " << double(1000000 * fullTime / double(full.rows()*full.cols()*nReps)) << " mus/element" << endl;

      gttic_(topTime);
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t j=0; j<(size_t)top.cols(); ++j)
          for(size_t i=0; i<(size_t)top.rows(); ++i)
            top(i,j) = uniform(rng);
      gttoc_(topTime);
      tictoc_getNode(topTimeNode, topTime);
      topTime = topTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Top:   " << double(1000000 * topTime / double(top.rows()*top.cols()*nReps)) << " mus/element" << endl;

      gttic_(blockTime);
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t j=0; j<(size_t)block.cols(); ++j)
          for(size_t i=0; i<(size_t)block.rows(); ++i)
            block(i,j) = uniform(rng);
      gttoc_(blockTime);
      tictoc_getNode(blockTimeNode, blockTime);
      blockTime = blockTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Block: " << double(1000000 * blockTime / double(block.rows()*block.cols()*nReps)) << " mus/element" << endl;

      cout << endl;
    }

    {
      double basicTime, fullTime, topTime, blockTime;
      typedef pair<size_t,size_t> ij_t;
      vector<ij_t> ijs(100000);

      cout << "Row-major matrix, random assignment:" << endl;

      // Do a few initial assignments to let any cache effects stabilize
      for_each(ijs.begin(), ijs.end(), _1 = make_pair(uniform_i(rng),uniform_j(rng)));
      for(size_t rep=0; rep<1000; ++rep)
        for(const ij_t& ij: ijs) { mat(ij.first, ij.second) = uniform(rng); }

      gttic_(basicTime);
      for_each(ijs.begin(), ijs.end(), _1 = make_pair(uniform_i(rng),uniform_j(rng)));
      for(size_t rep=0; rep<1000; ++rep)
        for(const ij_t& ij: ijs) { mat(ij.first, ij.second) = uniform(rng); }
      gttoc_(basicTime);
      tictoc_getNode(basicTimeNode, basicTime);
      basicTime = basicTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Basic: " << double(1000000 * basicTime / double(ijs.size()*nReps)) << " mus/element" << endl;

      gttic_(fullTime);
      for_each(ijs.begin(), ijs.end(), _1 = make_pair(uniform_i(rng),uniform_j(rng)));
      for(size_t rep=0; rep<1000; ++rep)
        for(const ij_t& ij: ijs) { full(ij.first, ij.second) = uniform(rng); }
      gttoc_(fullTime);
      tictoc_getNode(fullTimeNode, fullTime);
      fullTime = fullTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Full:  " << double(1000000 * fullTime / double(ijs.size()*nReps)) << " mus/element" << endl;

      gttic_(topTime);
      for_each(ijs.begin(), ijs.end(), _1 = make_pair(uniform_i(rng)%top.rows(),uniform_j(rng)));
      for(size_t rep=0; rep<1000; ++rep)
        for(const ij_t& ij: ijs) { top(ij.first, ij.second) = uniform(rng); }
      gttoc_(topTime);
      tictoc_getNode(topTimeNode, topTime);
      topTime = topTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Top:   " << double(1000000 * topTime / double(ijs.size()*nReps)) << " mus/element" << endl;

      gttic_(blockTime);
      for_each(ijs.begin(), ijs.end(), _1 = make_pair(uniform_i(rng)%block.rows(),uniform_j(rng)%block.cols()));
      for(size_t rep=0; rep<1000; ++rep)
        for(const ij_t& ij: ijs) { block(ij.first, ij.second) = uniform(rng); }
      gttoc_(blockTime);
      tictoc_getNode(blockTimeNode, blockTime);
      blockTime = blockTimeNode->secs();
      gtsam::tictoc_reset_();
      cout << "  Block: " << double(1000000 * blockTime / double(ijs.size()*nReps)) << " mus/element" << endl;

      cout << endl;
    }
  }

//  if(true) {
//    cout << "\nTesting square triangular matrices:" << endl;
//
////    typedef triangular_matrix<double, ublas::upper, ublas::column_major> triangular;
////    typedef ublas::matrix<double, ublas::column_major> matrix;
//    typedef MatrixXd matrix; // default col major
//
////    triangular tri(5,5);
//
//    matrix mat(5,5);
//    for(size_t j=0; j<(size_t)mat.cols(); ++j)
//      for(size_t i=0; i<(size_t)mat.rows(); ++i)
//        mat(i,j) = uniform(rng);
//
//    tri = ublas::triangular_adaptor<matrix, ublas::upper>(mat);
//    cout << "  Assigned from triangular adapter: " << tri << endl;
//
//    cout << "  Triangular adapter of mat: " << ublas::triangular_adaptor<matrix, ublas::upper>(mat) << endl;
//
//    for(size_t j=0; j<(size_t)mat.cols(); ++j)
//      for(size_t i=0; i<(size_t)mat.rows(); ++i)
//        mat(i,j) = uniform(rng);
//    mat = tri;
//    cout << "  Assign matrix from triangular: " << mat << endl;
//
//    for(size_t j=0; j<(size_t)mat.cols(); ++j)
//      for(size_t i=0; i<(size_t)mat.rows(); ++i)
//        mat(i,j) = uniform(rng);
//    (ublas::triangular_adaptor<matrix, ublas::upper>(mat)) = tri;
//    cout << "  Assign triangular adaptor from triangular: " << mat << endl;
//  }

//  {
//    cout << "\nTesting wide triangular matrices:" << endl;
//
//    typedef triangular_matrix<double, ublas::upper, ublas::column_major> triangular;
//    typedef ublas::matrix<double, ublas::column_major> matrix;
//
//    triangular tri(5,7);
//
//    matrix mat(5,7);
//    for(size_t j=0; j<(size_t)mat.cols(); ++j)
//      for(size_t i=0; i<(size_t)mat.rows(); ++i)
//        mat(i,j) = uniform(rng);
//
//    tri = ublas::triangular_adaptor<matrix, ublas::upper>(mat);
//    cout << "  Assigned from triangular adapter: " << tri << endl;
//
//    cout << "  Triangular adapter of mat: " << ublas::triangular_adaptor<matrix, ublas::upper>(mat) << endl;
//
//    for(size_t j=0; j<(size_t)mat.cols(); ++j)
//      for(size_t i=0; i<(size_t)mat.rows(); ++i)
//        mat(i,j) = uniform(rng);
//    mat = tri;
//    cout << "  Assign matrix from triangular: " << mat << endl;
//
//    for(size_t j=0; j<(size_t)mat.cols(); ++j)
//      for(size_t i=0; i<(size_t)mat.rows(); ++i)
//        mat(i,j) = uniform(rng);
//    mat = ublas::triangular_adaptor<matrix, ublas::upper>(mat);
//    cout << "  Assign matrix from triangular adaptor of self: " << mat << endl;
//  }

//  {
//    cout << "\nTesting subvectors:" << endl;
//
//    typedef MatrixXd matrix;
//    matrix mat(4,4);
//
//    for(size_t j=0; j<(size_t)mat.cols(); ++j)
//      for(size_t i=0; i<(size_t)mat.rows(); ++i)
//        mat(i,j) = i*mat.rows() + j;
//    cout << "  mat = " << mat;
//
//    cout << "  vec(1:4, 2:2) = " << mat.block(1,2, ), ublas::range(1,4), ublas::range(2,2));
//
//  }

  return 0;

}
