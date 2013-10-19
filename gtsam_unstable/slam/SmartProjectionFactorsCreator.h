/*
 * SmartProjectionFactorsCreator.h
 *
 *  Created on: Oct 8, 2013
 *  @author Zsolt Kira
 */

#ifndef SMARTPROJECTIONFACTORSCREATOR_H_
#define SMARTPROJECTIONFACTORSCREATOR_H_

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Use a map to store landmark/smart factor pairs
#include <gtsam/base/FastMap.h>
// #include <gtsam_unstable/slam/SmartProjectionFactor.h>
#include <gtsam_unstable/slam/SmartProjectionHessianFactor.h>

#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>

#include <fstream>
#include <iostream>
#include <utility>

namespace gtsam {

  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class SmartProjectionFactorsCreator {

  typedef SmartProjectionHessianFactor<Pose3, Point3, CALIBRATION> SmartFactor;
  typedef FastMap<Key, boost::shared_ptr<SmartProjectionHessianFactorState> > SmartFactorToStateMap;
  typedef FastMap<Key, boost::shared_ptr<SmartFactor> > SmartFactorMap;

  public:
    SmartProjectionFactorsCreator(const SharedNoiseModel& model,
        const boost::shared_ptr<CALIBRATION>& K,
        const double rankTol,
        const double linThreshold,
        boost::optional<POSE> body_P_sensor = boost::none) :
          noise_(model), K_(K), rankTolerance_(rankTol),
          linearizationThreshold_(linThreshold), body_P_sensor_(body_P_sensor),
          totalNumMeasurements(0), numLandmarks(0) {};

    void add(Key landmarkKey,
        Key poseKey, Point2 measurement, NonlinearFactorGraph &graph) {

      std::vector<Key> views;
      std::vector<Point2> measurements;

      bool debug = false;

      // Check if landmark exists in mapping
      SmartFactorToStateMap::iterator fsit = smartFactorStates.find(landmarkKey);
      typename SmartFactorMap::iterator fit = smartFactors.find(landmarkKey);
      if (fsit != smartFactorStates.end() && fit != smartFactors.end()) {
        if (debug) fprintf(stderr,"Adding measurement to existing landmark\n");

        // Add measurement to smart factor
        (*fit).second->add(measurement, poseKey, noise_, K_);
        totalNumMeasurements++;
        if (debug) (*fit).second->print();

      } else {

        if (debug) fprintf(stderr,"New landmark (%d,%d)\n", fsit != smartFactorStates.end(), fit != smartFactors.end());

        views.push_back(poseKey);
        measurements.push_back(measurement);

        // This is a new landmark, create a new factor and add to mapping
        boost::shared_ptr<SmartProjectionHessianFactorState> smartFactorState(new SmartProjectionHessianFactorState());
        boost::shared_ptr<SmartFactor> smartFactor(new SmartFactor(rankTolerance_, linearizationThreshold_));
        smartFactor->add(measurement, poseKey, noise_, K_);
        smartFactorStates.insert( std::make_pair(landmarkKey, smartFactorState) );
        smartFactors.insert( std::make_pair(landmarkKey, smartFactor) );
        graph.push_back(smartFactor);

        numLandmarks++;
        //landmarkKeys.push_back( L(l) );
        totalNumMeasurements++;

        views.clear();
        measurements.clear();
      }
    }

    void add(Key landmarkKey, Key poseKey,
        Point2 measurement,
        const SharedNoiseModel& model,
        const boost::shared_ptr<CALIBRATION>& Ki,
        NonlinearFactorGraph &graph) {

      std::vector<Key> views;
      std::vector<Point2> measurements;

      // std::cout << "matrix : " << K->K() << std::endl;

      bool debug = false;

      // Check if landmark exists in mapping
      SmartFactorToStateMap::iterator fsit = smartFactorStates.find(landmarkKey);
      typename SmartFactorMap::iterator fit = smartFactors.find(landmarkKey);
      if (fsit != smartFactorStates.end() && fit != smartFactors.end()) {
        if (debug) fprintf(stderr,"Adding measurement to existing landmark\n");
        // (*fit).second->print("before: ");
        // Add measurement to smart factor
        (*fit).second->add(measurement, poseKey, model, Ki);
        // (*fit).second->print("after: ");
        totalNumMeasurements++;
        if (debug) (*fit).second->print();

      } else {

        if (debug) std::cout <<"landmark " << DefaultKeyFormatter(landmarkKey) << "pose " << DefaultKeyFormatter(poseKey) << std::endl;

        // if (debug) fprintf(stderr,"landmarkKey %d poseKey %d measurement\n", landmarkKey, fit != smartFactors.end());

        // This is a new landmark, create a new factor and add to mapping
        boost::shared_ptr<SmartProjectionHessianFactorState> smartFactorState(new SmartProjectionHessianFactorState());
        boost::shared_ptr<SmartFactor> smartFactor(new SmartFactor(rankTolerance_, linearizationThreshold_));
        smartFactor->add(measurement, poseKey, model, Ki);
        // smartFactor->print("created: ");
        // smartFactor->print(" ");
        smartFactorStates.insert( std::make_pair(landmarkKey, smartFactorState) );
        smartFactors.insert( std::make_pair(landmarkKey, smartFactor) );
        graph.push_back(smartFactor);

        if (debug) std::cout <<" graph size " << graph.size() << std::endl;

        numLandmarks++;
        totalNumMeasurements++;

      }
    }

    unsigned int getTotalNumMeasurements() { return totalNumMeasurements; }
    unsigned int getNumLandmarks() { return numLandmarks; }

  protected:
    const SharedNoiseModel noise_;   ///< noise model used
    ///< (important that the order is the same as the keys that we use to create the factor)
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object

    double rankTolerance_; ///< threshold to decide whether triangulation is degenerate

    double linearizationThreshold_; ///< threshold to decide whether to re-linearize

    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    SmartFactorToStateMap smartFactorStates;
    SmartFactorMap smartFactors;

    unsigned int totalNumMeasurements;
    //landmarkKeys.push_back( L(l) );
    unsigned int numLandmarks;

  };

}

#endif /* SMARTPROJECTIONFACTORSCREATOR_H_ */
