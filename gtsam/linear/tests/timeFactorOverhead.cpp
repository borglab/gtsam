/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeFactorOverhead.cpp
 * @brief   Compares times of solving large single-factor graphs with their multi-factor equivalents.
 * @author  Richard Roberts
 * @date    Aug 20, 2010
 */

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SharedDiagonal.h>
#include <gtsam/inference/EliminationTree-inl.h>

#include <boost/random.hpp>
#include <boost/timer.hpp>
#include <vector>

using namespace gtsam;
using namespace std;

typedef EliminationTree<GaussianFactor> GaussianEliminationTree;

static boost::variate_generator<boost::mt19937, boost::uniform_real<> > rg(boost::mt19937(), boost::uniform_real<>(0.0, 1.0));

int main(int argc, char *argv[]) {

  Index key = 0;

  size_t vardim = 2;
  size_t blockdim = 1;
  size_t nBlocks = 2000;

  size_t nTrials = 10;

  double blockbuild, blocksolve, combbuild, combsolve;

  cout << "\n1 variable of dimension " << vardim << ", " <<
      nBlocks << " blocks of dimension " << blockdim << "\n";
  cout << nTrials << " trials\n";

  /////////////////////////////////////////////////////////////////////////////
  // Timing test with blockwise Gaussian factor graphs

  {
    // Build GFG's
    cout << "Building blockwise Gaussian factor graphs... ";
    cout.flush();
    boost::timer timer;
    timer.restart();
    vector<GaussianFactorGraph> blockGfgs;
    blockGfgs.reserve(nTrials);
    for(size_t trial=0; trial<nTrials; ++trial) {
      blockGfgs.push_back(GaussianFactorGraph());
      SharedDiagonal noise = noiseModel::Isotropic::Sigma(blockdim, 1.0);
      for(size_t i=0; i<nBlocks; ++i) {
        // Generate a random Gaussian factor
        Matrix A(blockdim, vardim);
        for(size_t j=0; j<blockdim; ++j)
          for(size_t k=0; k<vardim; ++k)
            A(j,k) = rg();
        Vector b(blockdim);
        for(size_t j=0; j<blockdim; ++j)
          b(j) = rg();
        blockGfgs[trial].push_back(JacobianFactor::shared_ptr(new JacobianFactor(key, A, b, noise)));
      }
    }
    blockbuild = timer.elapsed();
    cout << blockbuild << " s" << endl;

    // Solve GFG's
    cout << "Solving blockwise Gaussian factor graphs... ";
    cout.flush();
    timer.restart();
    for(size_t trial=0; trial<nTrials; ++trial) {
//      cout << "Trial " << trial << endl;
      GaussianBayesNet::shared_ptr gbn(GaussianEliminationTree::Create(blockGfgs[trial])->eliminate(&EliminateQR));
      VectorValues soln(optimize(*gbn));
    }
    blocksolve = timer.elapsed();
    cout << blocksolve << " s" << endl;
  }


  /////////////////////////////////////////////////////////////////////////////
  // Timing test with combined-factor Gaussian factor graphs

  {
    // Build GFG's
    cout << "Building combined-factor Gaussian factor graphs... ";
    cout.flush();
    boost::timer timer;
    timer.restart();
    vector<GaussianFactorGraph> combGfgs;
    for(size_t trial=0; trial<nTrials; ++trial) {
      combGfgs.push_back(GaussianFactorGraph());
      SharedDiagonal noise = noiseModel::Isotropic::Sigma(blockdim, 1.0);

      Matrix Acomb(blockdim*nBlocks, vardim);
      Vector bcomb(blockdim*nBlocks);
      for(size_t i=0; i<nBlocks; ++i) {
        // Generate a random Gaussian factor
        for(size_t j=0; j<blockdim; ++j)
          for(size_t k=0; k<vardim; ++k)
            Acomb(blockdim*i+j, k) = rg();
        Vector b(blockdim);
        for(size_t j=0; j<blockdim; ++j)
          bcomb(blockdim*i+j) = rg();
      }
      combGfgs[trial].push_back(JacobianFactor::shared_ptr(new JacobianFactor(key, Acomb, bcomb,
          noiseModel::Isotropic::Sigma(blockdim*nBlocks, 1.0))));
    }
    combbuild = timer.elapsed();
    cout << combbuild << " s" << endl;

    // Solve GFG's
    cout << "Solving combined-factor Gaussian factor graphs... ";
    cout.flush();
    timer.restart();
    for(size_t trial=0; trial<nTrials; ++trial) {
      GaussianBayesNet::shared_ptr gbn(GaussianEliminationTree::Create(combGfgs[trial])->eliminate(&EliminateQR));
      VectorValues soln(optimize(*gbn));
    }
    combsolve = timer.elapsed();
    cout << combsolve << " s" << endl;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Print per-graph times
  cout << "\nPer-factor-graph times for building and solving\n";
  cout << "Blockwise:  total " << (1000.0*(blockbuild+blocksolve)/double(nTrials)) <<
      "  build " << (1000.0*blockbuild/double(nTrials)) <<
      "  solve " << (1000.0*blocksolve/double(nTrials)) << " ms/graph\n";
  cout << "Combined:   total " << (1000.0*(combbuild+combsolve)/double(nTrials)) <<
      "  build " << (1000.0*combbuild/double(nTrials)) <<
      "  solve " << (1000.0*combsolve/double(nTrials)) << " ms/graph\n";
  cout << "Fraction of time spent in overhead\n" <<
      "  total " << (((blockbuild+blocksolve)-(combbuild+combsolve)) / (blockbuild+blocksolve)) << "\n" <<
      "  build " << ((blockbuild-combbuild) / blockbuild) << "\n" <<
      "  solve " << ((blocksolve-combsolve) / blocksolve) << "\n";
  cout << endl;

  return 0;
}

