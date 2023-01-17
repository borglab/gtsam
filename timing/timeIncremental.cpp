/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    timeIncremental.cpp
 * @brief   Overall timing tests for incremental solving
 * @author  Richard Roberts
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/base/timing.h>

#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/export.hpp>
#include <boost/range/adaptor/reversed.hpp>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

typedef Pose2 Pose;

typedef NoiseModelFactorN<Pose> NM1;
typedef NoiseModelFactorN<Pose,Pose> NM2;
typedef BearingRangeFactor<Pose,Point2> BR;

//GTSAM_VALUE_EXPORT(Value);
//GTSAM_VALUE_EXPORT(Pose);
//GTSAM_VALUE_EXPORT(NonlinearFactor);
//GTSAM_VALUE_EXPORT(NoiseModelFactor);
//GTSAM_VALUE_EXPORT(NM1);
//GTSAM_VALUE_EXPORT(NM2);
//GTSAM_VALUE_EXPORT(BetweenFactor<Pose>);
//GTSAM_VALUE_EXPORT(PriorFactor<Pose>);
//GTSAM_VALUE_EXPORT(BR);
//GTSAM_VALUE_EXPORT(noiseModel::Base);
//GTSAM_VALUE_EXPORT(noiseModel::Isotropic);
//GTSAM_VALUE_EXPORT(noiseModel::Gaussian);
//GTSAM_VALUE_EXPORT(noiseModel::Diagonal);
//GTSAM_VALUE_EXPORT(noiseModel::Unit);

double chi2_red(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& config) {
  // Compute degrees of freedom (observations - variables)
  // In ocaml, +1 was added to the observations to account for the prior, but
  // the factor graph already includes a factor for the prior/equality constraint.
  //  double dof = graph.size() - config.size();
  int graph_dim = 0;
  for(const std::shared_ptr<gtsam::NonlinearFactor>& nlf: graph) {
    graph_dim += nlf->dim();
  }
  double dof = graph_dim - config.dim(); // kaess: changed to dim
  return 2. * graph.error(config) / dof; // kaess: added factor 2, graph.error returns half of actual error
}

int main(int argc, char *argv[]) {

  cout << "Loading data..." << endl;

  gttic_(Find_datafile);
  //string datasetFile = findExampleDataFile("w10000");
  string datasetFile = findExampleDataFile("victoria_park");
  std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> data =
    load2D(datasetFile);
  gttoc_(Find_datafile);

  NonlinearFactorGraph measurements = *data.first;
  Values initial = *data.second;

  cout << "Playing forward time steps..." << endl;

  ISAM2 isam2;

  size_t nextMeasurement = 0;
  for(size_t step=1; nextMeasurement < measurements.size(); ++step) {

    Values newVariables;
    NonlinearFactorGraph newFactors;

    // Collect measurements and new variables for the current step
    gttic_(Collect_measurements);
    if(step == 1) {
      //      cout << "Initializing " << 0 << endl;
      newVariables.insert(0, Pose());
      // Add prior
      newFactors.addPrior(0, Pose(), noiseModel::Unit::Create(3));
    }
    while(nextMeasurement < measurements.size()) {

      NonlinearFactor::shared_ptr measurementf = measurements[nextMeasurement];

      if(BetweenFactor<Pose>::shared_ptr measurement =
        boost::dynamic_pointer_cast<BetweenFactor<Pose> >(measurementf))
      {
        // Stop collecting measurements that are for future steps
        if(measurement->key<1>() > step || measurement->key<2>() > step)
          break;

        // Require that one of the nodes is the current one
        if(measurement->key<1>() != step && measurement->key<2>() != step)
          throw runtime_error("Problem in data file, out-of-sequence measurements");

        // Add a new factor
        newFactors.push_back(measurement);

        // Initialize the new variable
        if(measurement->key<1>() == step && measurement->key<2>() == step-1) {
          if(step == 1)
            newVariables.insert(step, measurement->measured().inverse());
          else {
            Pose prevPose = isam2.calculateEstimate<Pose>(step-1);
            newVariables.insert(step, prevPose * measurement->measured().inverse());
          }
          //        cout << "Initializing " << step << endl;
        } else if(measurement->key<2>() == step && measurement->key<1>() == step-1) {
          if(step == 1)
            newVariables.insert(step, measurement->measured());
          else {
            Pose prevPose = isam2.calculateEstimate<Pose>(step-1);
            newVariables.insert(step, prevPose * measurement->measured());
          }
          //        cout << "Initializing " << step << endl;
        }
      }
      else if(BearingRangeFactor<Pose, Point2>::shared_ptr measurement =
        boost::dynamic_pointer_cast<BearingRangeFactor<Pose, Point2> >(measurementf))
      {
        Key poseKey = measurement->keys()[0], lmKey = measurement->keys()[1];

        // Stop collecting measurements that are for future steps
        if(poseKey > step)
          throw runtime_error("Problem in data file, out-of-sequence measurements");

        // Add new factor
        newFactors.push_back(measurement);

        // Initialize new landmark
        if(!isam2.getLinearizationPoint().exists(lmKey))
        {
          Pose pose = isam2.calculateEstimate<Pose>(poseKey);
          Rot2 measuredBearing = measurement->measured().bearing();
          double measuredRange = measurement->measured().range();
          newVariables.insert(lmKey,
            pose.transformFrom(measuredBearing.rotate(Point2(measuredRange, 0.0))));
        }
      }
      else
      {
        throw std::runtime_error("Unknown factor type read from data file");
      }
      ++ nextMeasurement;
    }
    gttoc_(Collect_measurements);

    // Update iSAM2
    gttic_(Update_ISAM2);
    isam2.update(newFactors, newVariables);
    gttoc_(Update_ISAM2);

    if(step % 100 == 0) {
      gttic_(chi2);
      Values estimate(isam2.calculateEstimate());
      double chi2 = chi2_red(isam2.getFactorsUnsafe(), estimate);
      cout << "chi2 = " << chi2 << endl;
      gttoc_(chi2);
    }

    tictoc_finishedIteration_();

    if(step % 1000 == 0) {
      cout << "Step " << step << endl;
      tictoc_print_();
    }
  }

  //try {
  //  {
  //    std::ofstream writerStream("incremental_init", ios::binary);
  //    boost::archive::binary_oarchive writer(writerStream);
  //    writer << isam2.calculateEstimate();
  //    writerStream.close();
  //  }
  //  {
  //    std::ofstream writerStream("incremental_graph", ios::binary);
  //    boost::archive::binary_oarchive writer(writerStream);
  //    writer << isam2.getFactorsUnsafe();
  //    writerStream.close();
  //  }
  //} catch(std::exception& e) {
  //  cout << e.what() << endl;
  //}

  NonlinearFactorGraph graph;
  Values values;

  //{
  //  std::ifstream readerStream("incremental_init", ios::binary);
  //  boost::archive::binary_iarchive reader(readerStream);
  //  reader >> values;
  //}
  //{
  //  std::ifstream readerStream("incremental_graph", ios::binary);
  //  boost::archive::binary_iarchive reader(readerStream);
  //  reader >> graph;
  //}

  graph = isam2.getFactorsUnsafe();
  values = isam2.calculateEstimate();

  // Compute marginals
  try {
    Marginals marginals(graph, values);
    int i=0;
    for (Key key1: boost::adaptors::reverse(values.keys())) {
      int j=0;
      for (Key key2: boost::adaptors::reverse(values.keys())) {
        if(i != j) {
          gttic_(jointMarginalInformation);
          KeyVector keys(2);
          keys[0] = key1;
          keys[1] = key2;
          JointMarginal info = marginals.jointMarginalInformation(keys);
          gttoc_(jointMarginalInformation);
          tictoc_finishedIteration_();
        }
        ++j;
        if(j >= 50)
          break;
      }
      ++i;
      if(i >= 50)
        break;
    }
    tictoc_print_();
    for(Key key: values.keys()) {
      gttic_(marginalInformation);
      Matrix info = marginals.marginalInformation(key);
      gttoc_(marginalInformation);
      tictoc_finishedIteration_();
      ++i;
    }
  } catch(std::exception& e) {
    cout << e.what() << endl;
  }
  tictoc_print_();

  return 0;
}
