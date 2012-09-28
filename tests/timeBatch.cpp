/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    timeBatch.cpp
 * @brief   Overall timing tests for batch solving
 * @author  Richard Roberts
 */

#include <gtsam/base/timing.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

int main(int argc, char *argv[]) {

  cout << "Loading data..." << endl;

  string datasetFile = findExampleDataFile("w10000-odom");
  std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> data =
    load2D(datasetFile);

  NonlinearFactorGraph graph = *data.first;
  Values initial = *data.second;

  cout << "Optimizing..." << endl;

  tic_(1, "Create optimizer");
  LevenbergMarquardtOptimizer optimizer(graph, initial);
  toc_(1, "Create optimizer");
  tictoc_print_();
  double lastError = optimizer.error();
  do {
    tic_(2, "Iterate optimizer");
    optimizer.iterate();
    toc_(2, "Iterate optimizer");
    tictoc_finishedIteration_();
    tictoc_print_();
    cout << "Error: " << optimizer.error() << ", lambda: " << optimizer.lambda() << endl;
  } while(!checkConvergence(optimizer.params().relativeErrorTol,
    optimizer.params().absoluteErrorTol, optimizer.params().errorTol,
    lastError, optimizer.error(), optimizer.params().verbosity));

  return 0;
}
