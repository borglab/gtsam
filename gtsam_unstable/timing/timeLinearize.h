/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeLinearize.h
 * @brief   time linearize
 * @author  Frank Dellaert
 * @date    October 10, 2014
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <time.h>
#include <iostream>
#include <iomanip>      // std::setprecision
using namespace std;
using namespace gtsam;

static const int n = 1000000;

void timeSingleThreaded(const string& str, const NonlinearFactor::shared_ptr& f,
    const Values& values) {
  long timeLog = clock();
  GaussianFactor::shared_ptr gf;
  for (int i = 0; i < n; i++)
    gf = f->linearize(values);
  long timeLog2 = clock();
  double seconds = (double) (timeLog2 - timeLog) / CLOCKS_PER_SEC;
  // cout << ((double) n / seconds) << " calls/second" << endl;
  cout << setprecision(3);
  cout << str << ((double) seconds * 1000000 / n) << " musecs/call" << endl;
}

void timeMultiThreaded(const string& str, const NonlinearFactor::shared_ptr& f,
    const Values& values) {
  NonlinearFactorGraph graph;
  for (int i = 0; i < n; i++)
    graph.push_back(f);
  long timeLog = clock();
  GaussianFactorGraph::shared_ptr gfg = graph.linearize(values);
  long timeLog2 = clock();
  double seconds = (double) (timeLog2 - timeLog) / CLOCKS_PER_SEC;
  // cout << ((double) n / seconds) << " calls/second" << endl;
  cout << setprecision(3);
  cout << str << ((double) seconds * 1000000 / n) << " musecs/call" << endl;
}

