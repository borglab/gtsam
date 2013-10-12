/*
 * SmartProjectionFactorsCreator.h
 *
 *  Created on: Oct 8, 2013
 *      Author: aspn
 */

#ifndef SMARTPROJECTIONFACTORSCREATOR_H_
#define SMARTPROJECTIONFACTORSCREATOR_H_

// Both relative poses and recovered trajectory poses will be stored as Pose3 objects
#include <gtsam/geometry/Pose3.h>

#include <gtsam/linear/NoiseModel.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Use a map to store landmark/smart factor pairs
#include <gtsam/base/FastMap.h>
#include <gtsam_unstable/slam/SmartProjectionFactor.h>

#include <gtsam/geometry/PinholeCamera.h>

#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <fstream>
#include <iostream>
#include <utility>

namespace gtsam {

  typedef SmartProjectionFactor<Pose3, Point3, Cal3_S2> SmartFactor;
  typedef FastMap<Key, boost::shared_ptr<SmartProjectionFactorState> > SmartFactorToStateMap;
  typedef FastMap<Key, boost::shared_ptr<SmartFactor> > SmartFactorMap;

  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class SmartProjectionFactorsCreator {
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
      SmartFactorMap::iterator fit = smartFactors.find(landmarkKey);
      if (fsit != smartFactorStates.end() && fit != smartFactors.end()) {
        if (debug) fprintf(stderr,"Adding measurement to existing landmark\n");

        // Add measurement to smart factor
        (*fit).second->add(measurement, poseKey);
        totalNumMeasurements++;
        if (debug) (*fit).second->print();

      } else {

        if (debug) fprintf(stderr,"New landmark (%d,%d)\n", fsit != smartFactorStates.end(), fit != smartFactors.end());

        views += poseKey;
        measurements += measurement;

        // This is a new landmark, create a new factor and add to mapping
        boost::shared_ptr<SmartProjectionFactorState> smartFactorState(new SmartProjectionFactorState());
        SmartFactor::shared_ptr smartFactor(new SmartFactor(views, measurements, noise_, K_, rankTolerance_, linearizationThreshold_));
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
    unsigned int numLandmarks;

  };

}

#endif /* SMARTPROJECTIONFACTORSCREATOR_H_ */
