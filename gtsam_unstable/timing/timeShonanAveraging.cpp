/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testShonanAveraging.cpp
 * @date   September 2019
 * @author Jing Wu
 * @brief  Unit tests for the timing of Shonan Averaging algorithm
 */

#include <gtsam_unstable/slam/ShonanAveraging.h>
#include <gtsam/base/timing.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <map>
#include <chrono>

using namespace std;
using namespace gtsam;

//string g2oFile = findExampleDataFile("toyExample.g2o");
// string g2oFile = "/Users/dellaert/git/SE-Sync/data/toy3D.g2o";
// string g2oFile = "/home/jingwu/catkin_workspace/gtsam/examples/Data/tinyGrid3D.g2o";

int main(int argc, char* argv[]) {
  // primitive argument parsing:
  if (argc > 3) {
    throw runtime_error("Usage: timeShonanAveraging  [g2oFile]");
  }

  string g2oFile;
  try {
    if (argc > 1)
      g2oFile = argv[argc - 1];
    else
//         g2oFile = "/home/jingwu/catkin_workspace/gtsam/examples/Data/toyExample.g2o";
        g2oFile = "/home/jingwu/Desktop/CS8903/SESync/data/SE3/tinyGrid3D.g2o";

    } catch (const exception& e) {
      cerr << e.what() << '\n';
      exit(1);
    }

  static const ShonanAveraging kShonan(g2oFile);
  auto graph = kShonan.buildGraphAt(3);

  // increase p value and try optimize using Shonan Algorithm. use chrono for timing
  // for (size_t p = 3; p < 15; p++){
  //   // cout << "*********************************************************" << endl;
  //   const Values initial = kShonan.initializeRandomlyAt(p);
  //   chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  //   for (size_t i = 0; i < 5; i++){
  //     const Values result = kShonan.tryOptimizingAt(p, initial);
  //     cout << "cost at SO(p)" << kShonan.costAt(p, result) << endl;
  //     const Values SO3Values = kShonan.projectFrom(p, result);
  //     cout << "cost at SO(3)" << kShonan.cost(SO3Values) << endl;
  //   }

  //   chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  //   chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 ) / 5;
  //   cout<<time_used.count()<<endl;
  // }

  // increase p value and try optimize using Shonan Algorithm. use tictoc() for timing
  for (size_t p = 3; p < 15; p++){
    // cout << "*********************************************************" << endl;
    const Values initial = kShonan.initializeRandomlyAt(p);
    gttic_(optimize);
    for (size_t i = 0; i < 5; i++){
      const Values result = kShonan.tryOptimizingAt(p, initial);
      cout << "cost at SO(p)" << kShonan.costAt(p, result) << endl;
      const Values SO3Values = kShonan.projectFrom(p, result);
      cout << "cost at SO(3)" << kShonan.cost(SO3Values) << endl;
    }
    tictoc_finishedIteration_();
    tictoc_print_();
  }
  
  return 0;
}
