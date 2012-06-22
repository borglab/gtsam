/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSLAMlike.cpp
 * @brief   Times solving of random SLAM-like graphs
 * @author  Richard Roberts
 * @date    Aug 30, 2010
 */

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SharedDiagonal.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

#include <boost/random.hpp>
#include <boost/timer.hpp>
#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <vector>

using namespace gtsam;
using namespace std;
using namespace boost::lambda;

typedef EliminationTree<JacobianFactor> GaussianEliminationTree;

static boost::variate_generator<boost::mt19937, boost::uniform_real<> > rg(boost::mt19937(), boost::uniform_real<>(0.0, 1.0));

bool _pair_compare(const pair<Index, Matrix>& a1, const pair<Index, Matrix>& a2) { return a1.first < a2.first; }

int main(int argc, char *argv[]) {

  size_t vardim = 3;
  size_t blockdim = 3;
  int nVars = 500;
  size_t blocksPerVar = 5;
  size_t varsPerBlock = 2;
  size_t varSpread = 10;

  size_t nTrials = 10;

  double blockbuild, blocksolve;

  cout << "\n" << nVars << " variables of dimension " << vardim << ", " <<
      blocksPerVar << " blocks for each variable, blocks of dimension " << blockdim << " measure " << varsPerBlock << " variables\n";
  cout << nTrials << " trials\n";

  boost::variate_generator<boost::mt19937, boost::uniform_int<> > ri(boost::mt19937(), boost::uniform_int<>(-varSpread, varSpread));

  /////////////////////////////////////////////////////////////////////////////
  // Timing test with blockwise Gaussian factor graphs

  {
    // Build GFG's
    cout << "Building SLAM-like Gaussian factor graphs... ";
    cout.flush();
    boost::timer timer;
    timer.restart();
    vector<GaussianFactorGraph> blockGfgs;
    blockGfgs.reserve(nTrials);
    for(size_t trial=0; trial<nTrials; ++trial) {
      blockGfgs.push_back(GaussianFactorGraph());
      SharedDiagonal noise = noiseModel::Isotropic::Sigma(blockdim, 1.0);
      for(int c=0; c<nVars; ++c) {
        for(size_t d=0; d<blocksPerVar; ++d) {
          vector<pair<Index, Matrix> > terms; terms.reserve(varsPerBlock);
          if(c == 0 && d == 0)
            // If it's the first factor, just make a prior
            terms.push_back(make_pair(0, eye(vardim)));
          else if(c != 0) {
            // Generate a random Gaussian factor
            for(size_t h=0; h<varsPerBlock; ++h) {
              int var;
              // If it's the first factor for this variable, make it "odometry"
              if(d == 0 && h == 0)
                var = c-1;
              else if(d == 0 && h == 1)
                var = c;
              else
                // Choose random variable ids
                do
                  var = c + ri();
                while(var < 0 || var > nVars-1 || find_if(terms.begin(), terms.end(),
                    boost::bind(&pair<Index, Matrix>::first, boost::lambda::_1) == Index(var)) != terms.end());
              Matrix A(blockdim, vardim);
              for(size_t j=0; j<blockdim; ++j)
                for(size_t k=0; k<vardim; ++k)
                  A(j,k) = rg();
              terms.push_back(make_pair(Index(var), A));
            }
          }
          Vector b(blockdim);
          sort(terms.begin(), terms.end(), &_pair_compare);
          for(size_t j=0; j<blockdim; ++j)
            b(j) = rg();
          if(!terms.empty())
            blockGfgs[trial].push_back(JacobianFactor::shared_ptr(new JacobianFactor(terms, b, noise)));
        }
      }

//      if(trial == 0)
//        blockGfgs.front().print("GFG: ");
    }
    blockbuild = timer.elapsed();
    cout << blockbuild << " s" << endl;

    // Solve GFG's
    cout << "Solving SLAM-like Gaussian factor graphs... ";
    cout.flush();
    timer.restart();
    for(size_t trial=0; trial<nTrials; ++trial) {
//      cout << "Trial " << trial << endl;
    	VectorValues soln(*GaussianMultifrontalSolver(blockGfgs[trial]).optimize());
    }
    blocksolve = timer.elapsed();
    cout << blocksolve << " s" << endl;
  }


  /////////////////////////////////////////////////////////////////////////////
  // Print per-graph times
  cout << "\nPer-factor-graph times for building and solving\n";
  cout << "  total " << (1000.0*(blockbuild+blocksolve)/double(nTrials)) <<
      "  build " << (1000.0*blockbuild/double(nTrials)) <<
      "  solve " << (1000.0*blocksolve/double(nTrials)) << " ms/graph\n";
  cout << endl;

  return 0;
}

/**
 * @file    timeSLAMlike.cpp
 * @brief   
 * @author  Richard Roberts
 * @date Aug 30, 2010
 */

