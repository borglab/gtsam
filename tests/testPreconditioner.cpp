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
  fg += JacobianFactor(2, -10*eye(2), 0, 10*eye(2), (Vector(2) << 2.0, -1.0).finished(), unit2);
  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg += JacobianFactor(2, -5*eye(2), 1, 5*eye(2), (Vector(2) << 0.0, 1.0).finished(), unit2);
  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg += JacobianFactor(0, -5*eye(2), 1, 5*eye(2), (Vector(2) << -1.0, 1.5).finished(), unit2);
  return fg;
}

/* ************************************************************************* */
static GaussianFactorGraph createSimpleGaussianFactorGraphUnordered() {
  GaussianFactorGraph fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_
  fg += JacobianFactor(2, 10*eye(2), -1.0*ones(2), unit2);
  // odometry between x1 and x2: x2-x1=[0.2;-0.1]
  fg += JacobianFactor(2, -10*eye(2), 1, 10*eye(2), (Vector(2) << 2.0, -1.0).finished(), unit2);
  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg += JacobianFactor(2, -5*eye(2), 1, 5*eye(2), (Vector(2) << 0.0, 1.0).finished(), unit2);
  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg += JacobianFactor(0, -5*eye(2), 1, 5*eye(2), (Vector(2) << -1.0, 1.5).finished(), unit2);
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
TEST( Preconditioner, buildBlocks2 ) {
  // Create simple Gaussian factor graph and initial values
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraphUnordered();

  // Expected Hessian block diagonal matrices
  std::map<Key, Matrix> expectedHessian =gfg.hessianBlockDiagonal();

  // Actual Hessian block diagonal matrices from BlockJacobiPreconditioner::build
  std::vector<Matrix> actualHessian = buildBlocks(gfg, KeyInfo(gfg));

  // Compare the number of block diagonal matrices
  EXPECT_LONGS_EQUAL(expectedHessian.size(), 3);
  EXPECT_LONGS_EQUAL(expectedHessian.size(), actualHessian.size());

  // Compare the values of matrices
  std::map<Key, Matrix>::const_iterator it1 = expectedHessian.begin();
  std::vector<Matrix>::const_iterator it2 = actualHessian.begin();
  for(; it1!=expectedHessian.end(); it1++, it2++)
    EXPECT(assert_equal(it1->second, *it2));
}

/* ************************************************************************* */
TEST( BlockJacobiPreconditioner, verySimpleLinerSystem) {
  // Ax = [4 1][u] = [1]  x0 = [2]
  //      [1 3][v]   [2]       [1]
  //
  // exact solution x = [1/11, 7/11]';
  //

  // Create a Gaussian Factor Graph
  GaussianFactorGraph simpleGFG;
  simpleGFG += JacobianFactor(0, (Matrix(2,2)<< 4, 1, 1, 3).finished(), (Vector(2) << 1, 2).finished(), noiseModel::Unit::Create(2));
  //simpleGFG.print("Factors\n");

  // Expected Hessian block diagonal matrices
  std::map<Key, Matrix> expectedHessian =simpleGFG.hessianBlockDiagonal();
  // Actual Hessian block diagonal matrices from BlockJacobiPreconditioner::build
  std::vector<Matrix> actualHessian = buildBlocks(simpleGFG, KeyInfo(simpleGFG));
  // Compare the number of block diagonal matrices
  EXPECT_LONGS_EQUAL(expectedHessian.size(), actualHessian.size());

  // Compare the values of matrices
  std::map<Key, Matrix>::const_iterator it1 = expectedHessian.begin();
  std::vector<Matrix>::const_iterator it2 = actualHessian.begin();

  // the function 'build' in BlockJacobianPreconditioner stores in 'buffer'
  // the cholesky decomposion of each block of the hessian (column major)
  // In this example there is a single block (i.e., a single value)
  // and the corresponding block of the Hessian is
  //
  // H0 = [17 7; 7 10]
  //
  Matrix expectedH0 = it1->second;
  Matrix actualH0 = *it2;
  EXPECT(assert_equal(expectedH0, (Matrix(2,2) << 17, 7, 7, 10).finished() ));
  EXPECT(assert_equal(expectedH0, actualH0));

  // The corresponding Cholesky decomposition is:
  // R = chol(H0) = [4.1231  1.6977   0   2.6679] (from Matlab)
  Preconditioner::shared_ptr preconditioner = createPreconditioner(boost::make_shared<gtsam::BlockJacobiPreconditionerParameters>());
  preconditioner->build(simpleGFG, KeyInfo(simpleGFG), std::map<Key,Vector>());
  boost::shared_ptr<BlockJacobiPreconditioner> blockJacobi = boost::dynamic_pointer_cast<BlockJacobiPreconditioner>(preconditioner);
  Vector expectedR = (Vector(4) << 4.1231, 0, 1.6977, 2.6679).finished();
  double* buf = blockJacobi->getBuffer();
  for(int i=0; i<4; ++i){
    EXPECT_DOUBLES_EQUAL(expectedR(i), buf[i], 1e-4);
  }

}

/* ************************************************************************* */
TEST( BlockJacobiPreconditioner, SimpleLinerSystem) {
  // Create a Gaussian Factor Graph
  GaussianFactorGraph simpleGFG;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  simpleGFG += JacobianFactor(2, (Matrix(2,2)<< 10, 0, 0, 10).finished(), (Vector(2) << -1, -1).finished(), unit2);
  simpleGFG += JacobianFactor(2, (Matrix(2,2)<< -10, 0, 0, -10).finished(), 0, (Matrix(2,2)<< 10, 0, 0, 10).finished(), (Vector(2) << 2, -1).finished(), unit2);
  simpleGFG += JacobianFactor(2, (Matrix(2,2)<< -5, 0, 0, -5).finished(), 1, (Matrix(2,2)<< 5, 0, 0, 5).finished(), (Vector(2) << 0, 1).finished(), unit2);
  simpleGFG += JacobianFactor(0, (Matrix(2,2)<< -5, 0, 0, -5).finished(), 1, (Matrix(2,2)<< 5, 0, 0, 5).finished(), (Vector(2) << -1, 1.5).finished(), unit2);
  simpleGFG += JacobianFactor(0, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  simpleGFG += JacobianFactor(1, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  simpleGFG += JacobianFactor(2, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);

  // Expected Hessian block diagonal matrices
  std::map<Key, Matrix> expectedHessian =simpleGFG.hessianBlockDiagonal();
  // Actual Hessian block diagonal matrices from BlockJacobiPreconditioner::build
  std::vector<Matrix> actualHessian = buildBlocks(simpleGFG, KeyInfo(simpleGFG));
  // Compare the number of block diagonal matrices
  EXPECT_LONGS_EQUAL(expectedHessian.size(), actualHessian.size());

  // Compare the values of matrices
  std::map<Key, Matrix>::const_iterator it1 = expectedHessian.begin();
  std::vector<Matrix>::const_iterator it2 = actualHessian.begin();


  Preconditioner::shared_ptr preconditioner = createPreconditioner(boost::make_shared<gtsam::BlockJacobiPreconditionerParameters>());
  preconditioner->build(simpleGFG, KeyInfo(simpleGFG), std::map<Key,Vector>());
  boost::shared_ptr<BlockJacobiPreconditioner> blockJacobi = boost::dynamic_pointer_cast<BlockJacobiPreconditioner>(preconditioner);
  double* buf = blockJacobi->getBuffer();
  for(size_t i=0; i<blockJacobi->getBufferSize(); ++i){
    std::cout << "buf[" << i << "] = " << buf[i] << std::endl;
  }

}

/* ************************************************************************* */
TEST( PCGsolver, verySimpleLinearSystem) {

  // Ax = [4 1][u] = [1]  x0 = [2]
  //      [1 3][v]   [2]       [1]
  //
  // exact solution x = [1/11, 7/11]';
  //

  // Create a Gaussian Factor Graph
  GaussianFactorGraph simpleGFG;
  simpleGFG += JacobianFactor(0, (Matrix(2,2)<< 4, 1, 1, 3).finished(), (Vector(2) << 1,2 ).finished(), noiseModel::Unit::Create(2));
  //simpleGFG.print("Factors\n");

  // Exact solution already known
  VectorValues exactSolution;
  exactSolution.insert(0, (Vector(2) << 1./11., 7./11.).finished());
  exactSolution.print("Exact");

  // Solve the system using direct method
  VectorValues deltaDirect = simpleGFG.optimize();
  EXPECT(assert_equal(exactSolution, deltaDirect, 1e-7));
  deltaDirect.print("Direct");

  // Solve the system using PCG
  // With Dummy preconditioner
  gtsam::PCGSolverParameters::shared_ptr pcg = boost::make_shared<gtsam::PCGSolverParameters>();
  pcg->preconditioner_ = boost::make_shared<gtsam::DummyPreconditionerParameters>();
  pcg->setMaxIterations(500);
  pcg->setEpsilon_abs(0.0);
  pcg->setEpsilon_rel(0.0);
  //pcg->setVerbosity("ERROR");
  VectorValues deltaPCGDummy = PCGSolver(*pcg).optimize(simpleGFG);
  EXPECT(assert_equal(exactSolution, deltaPCGDummy, 1e-7));
  deltaPCGDummy.print("PCG Dummy");

  // With Block-Jacobi preconditioner
  gtsam::PCGSolverParameters::shared_ptr pcgJacobi = boost::make_shared<gtsam::PCGSolverParameters>();
  pcgJacobi->preconditioner_ = boost::make_shared<gtsam::BlockJacobiPreconditionerParameters>();
  pcgJacobi->setMaxIterations(1500);// It takes more than 1000 iterations for this test
  pcgJacobi->setEpsilon_abs(0.0);
  pcgJacobi->setEpsilon_rel(0.0);
  VectorValues deltaPCGJacobi = PCGSolver(*pcgJacobi).optimize(simpleGFG);

  // Failed!
  EXPECT(assert_equal(exactSolution, deltaPCGJacobi, 1e-5));
  deltaPCGJacobi.print("PCG Jacobi");
}

/* ************************************************************************* */
TEST(PCGSolver, simpleLinearSystem) {
  // Create a Gaussian Factor Graph
  GaussianFactorGraph simpleGFG;
  //SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  SharedDiagonal unit2 = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.3));
  simpleGFG += JacobianFactor(2, (Matrix(2,2)<< 10, 0, 0, 10).finished(), (Vector(2) << -1, -1).finished(), unit2);
  simpleGFG += JacobianFactor(2, (Matrix(2,2)<< -10, 0, 0, -10).finished(), 0, (Matrix(2,2)<< 10, 0, 0, 10).finished(), (Vector(2) << 2, -1).finished(), unit2);
  simpleGFG += JacobianFactor(2, (Matrix(2,2)<< -5, 0, 0, -5).finished(), 1, (Matrix(2,2)<< 5, 0, 0, 5).finished(), (Vector(2) << 0, 1).finished(), unit2);
  simpleGFG += JacobianFactor(0, (Matrix(2,2)<< -5, 0, 0, -5).finished(), 1, (Matrix(2,2)<< 5, 0, 0, 5).finished(), (Vector(2) << -1, 1.5).finished(), unit2);
  simpleGFG += JacobianFactor(0, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  simpleGFG += JacobianFactor(1, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  simpleGFG += JacobianFactor(2, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  //simpleGFG.print("Factors\n");

  // Expected solution
  VectorValues expectedSolution;
  expectedSolution.insert(0, (Vector(2) << 0.100498, -0.196756).finished());
  expectedSolution.insert(2, (Vector(2) << -0.0990413, -0.0980577).finished());
  expectedSolution.insert(1, (Vector(2) << -0.0973252, 0.100582).finished());

  // Solve the system using direct method
  VectorValues deltaDirect = simpleGFG.optimize();
  EXPECT(assert_equal(expectedSolution, deltaDirect, 1e-5));
  expectedSolution.print("Expected");
  deltaDirect.print("Direct");

  // Solve the system using PCG
  VectorValues initial;
  initial.insert(0, (Vector(2) << 0.1, -0.1).finished());
  initial.insert(1, (Vector(2) << -0.1, 0.1).finished());
  initial.insert(2, (Vector(2) << -0.1, -0.1).finished());

  // With Dummy preconditioner
  gtsam::PCGSolverParameters::shared_ptr pcg = boost::make_shared<gtsam::PCGSolverParameters>();
  pcg->preconditioner_ = boost::make_shared<gtsam::DummyPreconditionerParameters>();
  pcg->setMaxIterations(500);
  pcg->setEpsilon_abs(0.0);
  pcg->setEpsilon_rel(0.0);
  //pcg->setVerbosity("ERROR");

  VectorValues deltaPCGDummy = PCGSolver(*pcg).optimize(simpleGFG, KeyInfo(simpleGFG), std::map<Key,Vector>(), initial);
  // Failed!
  EXPECT(assert_equal(expectedSolution, deltaPCGDummy, 1e-5));
  deltaPCGDummy.print("PCG Dummy");

  // Solve the system using Preconditioned Conjugate Gradient
  pcg->preconditioner_ = boost::make_shared<gtsam::BlockJacobiPreconditionerParameters>();
  VectorValues deltaPCGJacobi = PCGSolver(*pcg).optimize(simpleGFG, KeyInfo(simpleGFG), std::map<Key,Vector>(), initial);
  // Failed!
  EXPECT(assert_equal(expectedSolution, deltaPCGJacobi, 1e-5));
  deltaPCGJacobi.print("PCG Jacobi");

  // Test that the retrieval of the diagonal blocks of the Jacobian are correct.
  std::map<Key, Matrix> expectedHessian =simpleGFG.hessianBlockDiagonal();
  std::vector<Matrix> actualHessian = buildBlocks(simpleGFG, KeyInfo(simpleGFG));
  EXPECT_LONGS_EQUAL(expectedHessian.size(), actualHessian.size());
  std::map<Key, Matrix>::const_iterator it1 = expectedHessian.begin();
  std::vector<Matrix>::const_iterator it2 = actualHessian.begin();

  // The corresponding Cholesky decomposition is:
  // R = chol(H0) = [4.1231  1.6977   0   2.6679] (from Matlab)
  Preconditioner::shared_ptr preconditioner = createPreconditioner(boost::make_shared<gtsam::BlockJacobiPreconditionerParameters>());
  preconditioner->build(simpleGFG, KeyInfo(simpleGFG), std::map<Key,Vector>());
  boost::shared_ptr<BlockJacobiPreconditioner> blockJacobi = boost::dynamic_pointer_cast<BlockJacobiPreconditioner>(preconditioner);
  double* buf = blockJacobi->getBuffer();
  for(int i=0; i<4; ++i){}
  // TODO: EXPECT(assert_equal(number..,buf[i]));

  size_t i = 0;
  for(; it1!=expectedHessian.end(); it1++, it2++){
    //EXPECT(assert_equal(it1->second, *it2));
    Matrix R_i(2,2);
    R_i(0,0) = buf[i+0];
    R_i(0,1) = buf[i+1];
    R_i(1,0) = buf[i+2];
    R_i(1,1) = buf[i+3];

    Matrix actualH_i = R_i.transpose() * R_i;// i-th diagonal block
    EXPECT(assert_equal(it1->second, actualH_i));
    i += 4;
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
