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

#include <gtsam/base/timing.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>

#include <random>
#include <vector>

using namespace gtsam;
using namespace std;

static std::mt19937 rng;
static std::uniform_real_distribution<> uniform(0.0, 1.0);

int main(int argc, char *argv[]) {

  Key key = 0;

  size_t vardim = 2;
  size_t blockdim = 1;
  size_t nBlocks = 4000;

  size_t nTrials = 20;

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
    gttic_(blockbuild);
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
            A(j,k) = uniform(rng);
        Vector b(blockdim);
        for(size_t j=0; j<blockdim; ++j)
          b(j) = uniform(rng);
        blockGfgs[trial].push_back(std::make_shared<JacobianFactor>(key, A, b, noise));
      }
    }
    gttoc_(blockbuild);
    tictoc_getNode(blockbuildNode, blockbuild);
    blockbuild = blockbuildNode->secs();
    cout << blockbuild << " s" << endl;

    // Solve GFG's
    cout << "Solving blockwise Gaussian factor graphs... ";
    cout.flush();
    gttic_(blocksolve);
    for(size_t trial=0; trial<nTrials; ++trial) {
//      cout << "Trial " << trial << endl;
      GaussianBayesNet::shared_ptr gbn = blockGfgs[trial].eliminateSequential();
      VectorValues soln = gbn->optimize();
    }
    gttoc_(blocksolve);
    tictoc_getNode(blocksolveNode, blocksolve);
    blocksolve = blocksolveNode->secs();
    cout << blocksolve << " s" << endl;
  }


  /////////////////////////////////////////////////////////////////////////////
  // Timing test with combined-factor Gaussian factor graphs

  {
    // Build GFG's
    cout << "Building combined-factor Gaussian factor graphs... ";
    cout.flush();
    gttic_(combbuild);
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
            Acomb(blockdim*i+j, k) = uniform(rng);
        Vector b(blockdim);
        for(size_t j=0; j<blockdim; ++j)
          bcomb(blockdim*i+j) = uniform(rng);
      }
      combGfgs[trial].push_back(std::make_shared<JacobianFactor>(key, Acomb, bcomb,
          noiseModel::Isotropic::Sigma(blockdim*nBlocks, 1.0)));
    }
    gttoc(combbuild);
    tictoc_getNode(combbuildNode, combbuild);
    combbuild = combbuildNode->secs();
    cout << combbuild << " s" << endl;

    // Solve GFG's
    cout << "Solving combined-factor Gaussian factor graphs... ";
    cout.flush();
    gttic_(combsolve);
    for(size_t trial=0; trial<nTrials; ++trial) {
      GaussianBayesNet::shared_ptr gbn = combGfgs[trial].eliminateSequential();
      VectorValues soln = gbn->optimize();
    }
    gttoc_(combsolve);
    tictoc_getNode(combsolveNode, combsolve);
    combsolve = combsolveNode->secs();
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

