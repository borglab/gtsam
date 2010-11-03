/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSequentialOnDataset.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 7, 2010
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
  if(argc > 1)
    datasetname = argv[1];
  else
    datasetname = "intel";

  pair<shared_ptr<Pose2Graph>, shared_ptr<Pose2Values> > data = load2D(dataset(datasetname));

  tic_("Z 1 order");
  Ordering::shared_ptr ordering(data.first->orderingCOLAMD(*data.second));
  toc_("Z 1 order");
  tictoc_print_();

  tic_("Z 2 linearize");
  GaussianFactorGraph::shared_ptr gfg(data.first->linearize(*data.second, *ordering));
  toc_("Z 2 linearize");
  tictoc_print_();

  for(size_t trial = 0; trial < 10; ++trial) {

    tic_("Z 3 solve");
    VectorValues soln(*GaussianMultifrontalSolver(*gfg).optimize());
    toc_("Z 3 solve");

    tictoc_print_();
  }

  return 0;

}
