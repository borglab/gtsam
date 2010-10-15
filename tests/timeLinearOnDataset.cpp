/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeLinearOnDataset.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 7, 2010
 */

#include <gtsam/base/timing.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>

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

  tic("Z 1 order");
  Ordering::shared_ptr ordering(data.first->orderingCOLAMD(*data.second));
  toc("Z 1 order");

  tic("Z 2 linearize");
  GaussianFactorGraph::shared_ptr gfg(data.first->linearize(*data.second, *ordering));
  toc("Z 2 linearize");

  for(size_t trial = 0; trial < 100; ++trial) {

    tic("Z 3 solve");
    GaussianJunctionTree gjt(*gfg);
    VectorValues soln(gjt.optimize());
    toc("Z 3 solve");

    tictoc_print();
  }

  return 0;

}
