/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testPreconditioner.cpp
 *  @brief  Unit tests for Preconditioners
 *  @author Sungtae An
 *  @date   Nov 6, 2014
 **/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/linear/PCGSolver.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
static GaussianFactorGraph createSimpleGaussianFactorGraph() {
  GaussianFactorGraph fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
  fg += JacobianFactor(2, 10*eye(2), -1.0*ones(2), unit2);
  // odometry between x1 and x2: x2-x1=[0.2;-0.1]
  fg += JacobianFactor(2, -10*eye(2), 0, 10*eye(2), (Vector(2) << 2.0, -1.0), unit2);
  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg += JacobianFactor(2, -5*eye(2), 1, 5*eye(2), (Vector(2) << 0.0, 1.0), unit2);
  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg += JacobianFactor(0, -5*eye(2), 1, 5*eye(2), (Vector(2) << -1.0, 1.5), unit2);
  return fg;
}

/* ************************************************************************* */
// Copy of BlockJacobiPreconditioner::build
std::vector<Matrix> buildBlocks( const GaussianFactorGraph &gfg, const KeyInfo &keyInfo)
{
  const size_t n = keyInfo.size();
  std::vector<size_t> dims_ = keyInfo.colSpec();

  /* prepare the buffer of block diagonals */
  std::vector<Matrix> blocks; blocks.reserve(n);

  /* allocate memory for the factorization of block diagonals */
  size_t nnz = 0;
  for ( size_t i = 0 ; i < n ; ++i ) {
    const size_t dim = dims_[i];
    blocks.push_back(Matrix::Zero(dim, dim));
    // nnz += (((dim)*(dim+1)) >> 1); // d*(d+1) / 2  ;
    nnz += dim*dim;
  }

  /* compute the block diagonal by scanning over the factors */
  BOOST_FOREACH ( const GaussianFactor::shared_ptr &gf, gfg ) {
    if ( JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(gf) ) {
      for ( JacobianFactor::const_iterator it = jf->begin() ; it != jf->end() ; ++it ) {
        const KeyInfoEntry &entry =  keyInfo.find(*it)->second;
        const Matrix &Ai = jf->getA(it);
        blocks[entry.index()] += (Ai.transpose() * Ai);
      }
    }
    else if ( HessianFactor::shared_ptr hf = boost::dynamic_pointer_cast<HessianFactor>(gf) ) {
      for ( HessianFactor::const_iterator it = hf->begin() ; it != hf->end() ; ++it ) {
        const KeyInfoEntry &entry =  keyInfo.find(*it)->second;
        const Matrix &Hii = hf->info(it, it).selfadjointView();
        blocks[entry.index()] += Hii;
      }
    }
    else {
      throw invalid_argument("BlockJacobiPreconditioner::build gfg contains a factor that is neither a JacobianFactor nor a HessianFactor.");
    }
  }

  return blocks;
}

/* ************************************************************************* */
TEST( Preconditioner, buildBlocks ) {
  // Create simple Gaussian factor graph and initial values
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();
  Values initial;
  initial.insert(0,Point2(4, 5));
  initial.insert(1,Point2(0,  1));
  initial.insert(2,Point2(-5, 7));

  // Expected Hessian block diagonal matrices
  std::map<Key, Matrix> expectedHessian =gfg.hessianBlockDiagonal();

  // Actual Hessian block diagonal matrices from BlockJacobiPreconditioner::build
  std::vector<Matrix> actualHessian = buildBlocks(gfg, KeyInfo(gfg));

  // Compare the number of block diagonal matrices
  EXPECT_LONGS_EQUAL(expectedHessian.size(), actualHessian.size());

  // Compare the values of matrices
  std::map<Key, Matrix>::const_iterator it1 = expectedHessian.begin();
  std::vector<Matrix>::const_iterator it2 = actualHessian.begin();
  for(; it1!=expectedHessian.end(); it1++, it2++)
    EXPECT(assert_equal(it1->second, *it2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
