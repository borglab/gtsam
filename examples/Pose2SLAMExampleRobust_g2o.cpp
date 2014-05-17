/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample.cpp
 * @brief A 2D Pose SLAM example that reads input from g2o and uses robust kernels in optimization
 * @date May 15, 2014
 * @author Luca Carlone
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <fstream>
#include <sstream>

using namespace std;
using namespace gtsam;

#define LINESIZE 81920

int main(const int argc, const char *argv[]){

  if (argc < 2)
    std::cout << "Please specify input file (in g2o format) and output file" << std::endl;
  const string g2oFile = argv[1];

  ifstream is(g2oFile.c_str());
  if (!is)
    throw std::invalid_argument("File not found!");

  std::cout << "Reading g2o file: " << g2oFile << std::endl;
  // READ INITIAL GUESS FROM G2O FILE
  Values initial;
  string tag;
  while (is) {
    if(! (is >> tag))
      break;

    if (tag == "VERTEX_SE2") {
      int id;
      double x, y, yaw;
      is >> id >> x >> y >> yaw;
      initial.insert(id, Pose2(x, y, yaw));
    }
    is.ignore(LINESIZE, '\n');
  }
  is.clear(); /* clears the end-of-file and error flags */
  is.seekg(0, ios::beg);
  // initial.print("initial guess");

  // READ MEASUREMENTS FROM G2O FILE
  NonlinearFactorGraph graph;
  while (is) {
    if(! (is >> tag))
      break;

    if (tag == "EDGE_SE2") {
      int id1, id2;
      double x, y, yaw;
      double I11, I12, I13, I22, I23, I33;

      is >> id1 >> id2 >> x >> y >> yaw;
      is >> I11 >> I12 >> I13 >> I22 >> I23 >> I33;

      // Try to guess covariance matrix layout
      Matrix m(3,3);
      m << I11, I12, I13,  I12, I22, I23,  I13, I23, I33;

      Pose2 l1Xl2(x, y, yaw);

      noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Variances((Vector(3) << 1/I11, 1/I22, 1/I33));

      NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose2>(id1, id2, l1Xl2,
          noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), model)));
      graph.add(factor);
    }
    is.ignore(LINESIZE, '\n');
  }

  // otherwise GTSAM cannot solve the problem
  NonlinearFactorGraph graphWithPrior = graph;
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances((Vector(3) << 0.01, 0.01, 0.001));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));

  // GaussNewtonParams parameters;
  // Stop iterating once the change in error between steps is less than this value
  // parameters.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  // parameters.maxIterations = 100;
  // Create the optimizer ...
  std::cout << "Optimizing the factor graph" << std::endl;
  GaussNewtonOptimizer optimizer(graphWithPrior, initial); // , parameters);
  // ... and optimize
  Values result = optimizer.optimize();
  // result.print("results");
  std::cout << "Optimization complete" << std::endl;

  const string outputFile = argv[2];
  std::cout << "Writing results to file: " << outputFile << std::endl;

  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.0, 0.0, 0.0));
  save2D(graph, result, model, outputFile);

  return 0;
}
