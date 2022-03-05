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
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

int main(int argc, char *argv[]) {

  try {

    cout << "Loading data..." << endl;

    string datasetFile = findExampleDataFile("w10000");
    std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> data =
      load2D(datasetFile);

    NonlinearFactorGraph graph = *data.first;
    Values initial = *data.second;

    cout << "Optimizing..." << endl;

    gttic_(Create_optimizer);
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    gttoc_(Create_optimizer);
    tictoc_print_();
    double lastError = optimizer.error();
    do {
      gttic_(Iterate_optimizer);
      optimizer.iterate();
      gttoc_(Iterate_optimizer);
      tictoc_finishedIteration_();
      tictoc_print_();
      cout << "Error: " << optimizer.error() << ", lambda: " << optimizer.lambda() << endl;
      break;
    } while(!checkConvergence(optimizer.params().relativeErrorTol,
      optimizer.params().absoluteErrorTol, optimizer.params().errorTol,
      lastError, optimizer.error(), optimizer.params().verbosity));

    // Compute marginals
    Marginals marginals(graph, optimizer.values());
    int i=0;
    for(Key key: initial.keys()) {
      gttic_(marginalInformation);
      Matrix info = marginals.marginalInformation(key);
      gttoc_(marginalInformation);
      tictoc_finishedIteration_();
      if(i % 1000 == 0)
        tictoc_print_();
      ++i;
    }

  } catch(std::exception& e) {
    cout << e.what() << endl;
    return 1;
  }

  return 0;
}
