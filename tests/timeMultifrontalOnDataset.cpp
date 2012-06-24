/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSequentialOnDataset.cpp
 * @author  Richard Roberts
 * @date    Oct 7, 2010
 */

#include <gtsam/base/timing.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>

using namespace std;
using namespace gtsam;
using namespace boost;

int main(int argc, char *argv[]) {

  string datasetname;
  bool soft_prior = true;
  if(argc > 1)
    datasetname = argv[1];
  else
    datasetname = "intel";

  // check if there should be a constraint
  if (argc == 3 && string(argv[2]).compare("-c") == 0)
  	soft_prior = false;

  // find the number of trials - default is 10
  size_t nrTrials = 10;
  if (argc == 3 && string(argv[2]).compare("-c") != 0)
  	nrTrials = strtoul(argv[2], NULL, 10);
  else if (argc == 4)
  	nrTrials = strtoul(argv[3], NULL, 10);

  pair<boost::shared_ptr<pose2SLAM::Graph>, boost::shared_ptr<Values> > data = load2D(dataset(datasetname));

  // Add a prior on the first pose
  if (soft_prior)
  	data.first->addPosePrior(0, Pose2(), noiseModel::Isotropic::Sigma(Pose2::Dim(), 0.0005));
  else
  	data.first->addPoseConstraint(0, Pose2());

  tic_(1, "order");
  Ordering::shared_ptr ordering(data.first->orderingCOLAMD(*data.second));
  toc_(1, "order");
  tictoc_print_();

  tic_(2, "linearize");
  GaussianFactorGraph::shared_ptr gfg(data.first->linearize(*data.second, *ordering)->dynamicCastFactors<GaussianFactorGraph>());
  toc_(2, "linearize");
  tictoc_print_();

  for(size_t trial = 0; trial < nrTrials; ++trial) {

    tic_(3, "solve");
    tic(1, "construct solver");
    GaussianMultifrontalSolver solver(*gfg);
    toc(1, "construct solver");
    tic(2, "optimize");
    VectorValues soln(*solver.optimize());
    toc(2, "optimize");
    toc_(3, "solve");

    tictoc_print_();
  }

  return 0;

}
