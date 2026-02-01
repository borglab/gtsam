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
#include <iomanip>

static const int n = 1000000;

void timeSingleThreaded(const std::string& str,
    const gtsam::NonlinearFactor::shared_ptr& f, const gtsam::Values& values) {
  long timeLog = clock();
  gtsam::GaussianFactor::shared_ptr gf;
  for (int i = 0; i < n; i++)
    gf = f->linearize(values);
  long timeLog2 = clock();
  double seconds = (double) (timeLog2 - timeLog) / CLOCKS_PER_SEC;
  std::cout << std::setprecision(3);
  std::cout << str << ((double) seconds * 1000000 / n) << " musecs/call"
      << std::endl;
}

void timeMultiThreaded(const std::string& str,
    const gtsam::NonlinearFactor::shared_ptr& f, const gtsam::Values& values) {
  gtsam::NonlinearFactorGraph graph;
  for (int i = 0; i < n; i++)
    graph.push_back(f);
  long timeLog = clock();
  gtsam::GaussianFactorGraph::shared_ptr gfg = graph.linearize(values);
  long timeLog2 = clock();
  double seconds = (double) (timeLog2 - timeLog) / CLOCKS_PER_SEC;
  std::cout << std::setprecision(3);
  std::cout << str << ((double) seconds * 1000000 / n) << " musecs/call"
      << std::endl;
}

