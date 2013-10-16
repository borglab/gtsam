/*
 * GenericProjectionFactorsCreator.h
 *
 *  Created on: Oct 10, 2013
 *      Author: zkira
 */

#ifndef GENERICPROJECTIONFACTORSCREATOR_H_
#define GENERICPROJECTIONFACTORSCREATOR_H_

// Both relative poses and recovered trajectory poses will be stored as Pose3 objects
#include <gtsam/geometry/Pose3.h>

#include <gtsam/linear/NoiseModel.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Use a map to store landmark/smart factor pairs
#include <gtsam/base/FastMap.h>
#include <gtsam/slam/ProjectionFactor.h>


#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
//#include <boost/foreach.hpp>
//#include <boost/assign.hpp>
//#include <boost/assign/std/vector.hpp>
//#include <fstream>
//#include <iostream>
#include <utility>

namespace gtsam {


  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class GenericProjectionFactorsCreator {

    typedef GenericProjectionFactor<Pose3, Point3, CALIBRATION> ProjectionFactor;
    typedef FastMap<Key, std::vector<boost::shared_ptr<ProjectionFactor> > > ProjectionFactorMap;
    typedef FastMap<Key, boost::shared_ptr<CALIBRATION> > CalibrationMap;
    typedef FastMap<Key, int> OrderingMap;

  public:
    GenericProjectionFactorsCreator(const SharedNoiseModel& model,
        const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<POSE> body_P_sensor = boost::none) :
          noise_(model), K_(K), body_P_sensor_(body_P_sensor),
          orderingMethod(0), totalNumMeasurements(0), numLandmarks(0) {
        ordering = boost::make_shared<Ordering>(*(new Ordering()));
    };

    void add(Key landmarkKey,
        Key poseKey, Point2 measurement, NonlinearFactorGraph &graph) {
        bool debug = false;

        // Create projection factor
        boost::shared_ptr<ProjectionFactor> projectionFactor(new ProjectionFactor(measurement, noise_, poseKey, landmarkKey, K_));

        // Check if landmark exists in mapping
        typename ProjectionFactorMap::iterator pfit = projectionFactors.find(landmarkKey);
        if (pfit != projectionFactors.end()) {
          if (debug) fprintf(stderr,"Adding measurement to existing landmark\n");

          // Add projection factor to list of projection factors associated with this landmark
          (*pfit).second.push_back(projectionFactor);

        } else {
          if (debug) fprintf(stderr,"New landmark (%d)\n", pfit != projectionFactors.end());

          // Create a new vector of projection factors
          std::vector<boost::shared_ptr<ProjectionFactor> > projectionFactorVector;
          projectionFactorVector.push_back(projectionFactor);

          // Insert projection factor to NEW list of projection factors associated with this landmark
          projectionFactors.insert( std::make_pair(landmarkKey, projectionFactorVector) );

          // Add projection factor to graph
          //graph.push_back(projectionFactor);

          // We have a new landmark
          numLandmarks++;
          landmarkKeys.push_back( landmarkKey );
        }
    }

    void add(Key landmarkKey,
            Key poseKey,
            Point2 measurement,
            const SharedNoiseModel& model,
            const boost::shared_ptr<CALIBRATION>& K,
            NonlinearFactorGraph &graph) {
            bool debug = false;

            // Check if landmark exists in mapping
            typename CalibrationMap::iterator calfit = keyCalibrationMap.find(poseKey);
            if (calfit == keyCalibrationMap.end()){
              keyCalibrationMap.insert( std::make_pair(poseKey, K) );
            }

            // Create projection factor
            typename ProjectionFactor::shared_ptr projectionFactor(new ProjectionFactor(measurement, model, poseKey, landmarkKey, K));

            // Check if landmark exists in mapping
            typename ProjectionFactorMap::iterator pfit = projectionFactors.find(landmarkKey);
            if (pfit != projectionFactors.end()) {
              if (debug) fprintf(stderr,"Adding measurement to existing landmark\n");

              // Add projection factor to list of projection factors associated with this landmark
              (*pfit).second.push_back(projectionFactor);

            } else {
              if (debug) fprintf(stderr,"New landmark (%d)\n", pfit != projectionFactors.end());

              // Create a new vector of projection factors
              std::vector<boost::shared_ptr<ProjectionFactor> > projectionFactorVector;
              projectionFactorVector.push_back(projectionFactor);

              // Insert projection factor to NEW list of projection factors associated with this landmark
              projectionFactors.insert( std::make_pair(landmarkKey, projectionFactorVector) );

              // Add projection factor to graph
              //graph.push_back(projectionFactor);

              // We have a new landmark
              numLandmarks++;
              landmarkKeys.push_back( landmarkKey );
            }
        }

    void update(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr inputValues, gtsam::Values::shared_ptr outputValues, bool doTriangualatePoints = true) {
        addTriangulatedLandmarks(graph, inputValues, outputValues, doTriangualatePoints);
        updateOrdering(graph);
    }

    unsigned int getTotalNumMeasurements() { return totalNumMeasurements; }
    unsigned int getNumLandmarks() { return numLandmarks; }
    unsigned int getNumPoses() { return cameraPoseKeys.size(); }
    boost::shared_ptr<Ordering> getOrdering() { return ordering; }

  protected:

    void updateTriangulations() {
    }

    void updateOrdering(NonlinearFactorGraph &graph) {
        bool debug = false;

        if (1||debug) std::cout << "Landmark Keys: " << landmarkKeys.size() << " Pose Keys: " << cameraPoseKeys.size() << std::endl;
        if (1||debug) std::cout << "Pose ordering: " << ordering->size() << std::endl;

        if (orderingMethod == 1) {
            OrderingMap orderingMap;
            // Add landmark keys first for ordering
            BOOST_FOREACH(const Key& key, landmarkKeys) {
                orderingMap.insert( std::make_pair(key, 1) );
            }
            //Ordering::iterator oit;
            BOOST_FOREACH(const Key& key, cameraPoseKeys) {
                orderingMap.insert( std::make_pair(key, 2) );
            }
            *ordering = graph.orderingCOLAMDConstrained(orderingMap);
        }

        if (1||debug) std::cout << "Optimizing landmark first " << ordering->size() << std::endl;
    }

    void addTriangulatedLandmarks(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr loadedValues,
        gtsam::Values::shared_ptr graphValues, bool doTriangualatePoints) {

      bool debug = false;

      if(doTriangualatePoints)
        std::cout << "Triangulating 3D points" << std::endl;
      else
        std::cout << "Reading initial guess for 3D points from file" << std::endl;

      double rankTolerance=1;

      std::cout << "rankTolerance " << rankTolerance << std::endl;

      std::vector<boost::shared_ptr<ProjectionFactor> > projectionFactorVector;
      typename std::vector<boost::shared_ptr<ProjectionFactor> >::iterator vfit;
      Point3 point;
      Pose3 cameraPose;

      typename ProjectionFactorMap::iterator pfit;

      if (debug)  graphValues->print("graphValues \n");
      if (debug) std::cout << " # END VALUES: " << std::endl;

      // Iterate through all landmarks
      if (debug) std::cout << " PROJECTION FACTOR GROUPED: " << projectionFactors.size();
      int numProjectionFactors = 0;
      int numProjectionFactorsAdded = 0;
      int numFailures = 0;
      for (pfit = projectionFactors.begin(); pfit != projectionFactors.end(); pfit++) { // for each landmark!

        projectionFactorVector = (*pfit).second; // all factors connected to a given landmark

        std::vector<Pose3> cameraPoses;
        std::vector<Point2> measured;
        typename std::vector< boost::shared_ptr<CALIBRATION> > Ks;

        // Iterate through projection factors
        for (vfit = projectionFactorVector.begin(); vfit != projectionFactorVector.end(); vfit++) { // for each factors connected to the landmark
          numProjectionFactors++;

          if (debug) std::cout << "ProjectionFactor: " << std::endl;
          if (debug) (*vfit)->print("ProjectionFactor");

          // Iterate through poses // find calibration
          Key poseKey = (*vfit)->key1();
          typename CalibrationMap::iterator calfit = keyCalibrationMap.find(poseKey);
          if (calfit == keyCalibrationMap.end()){ // the pose is not there
            std::cout << "ProjectionFactor: " << std::endl;
          }else{
            Ks.push_back(calfit->second);
          }

          cameraPoses.push_back( loadedValues->at<Pose3>(poseKey) ); // get poses connected to the landmark
          measured.push_back( (*vfit)->measured() ); // get measurements of the landmark
        }

        // Triangulate landmark based on set of poses and measurements
        if (doTriangualatePoints){
          // std::cout << "Triangulating points " << std::endl;
          try {
            point = triangulatePoint3(cameraPoses, measured, Ks, rankTolerance);
            if (debug) std::cout << "Triangulation succeeded: " << point << std::endl;
          } catch( TriangulationUnderconstrainedException& e) {
            if (debug) std::cout << "Triangulation failed because of unconstrained exception" << std::endl;
            if (debug) {
              BOOST_FOREACH(const Pose3& pose, cameraPoses) {
                std::cout << " Pose: " << pose << std::endl;
              }
            }
            numFailures++;
            continue;
          } catch( TriangulationCheiralityException& e) {
            if (debug) std::cout << "Triangulation failed because of unconstrained exception" << std::endl;
            if (debug) {
              std::cout << "Triangulation failed because of cheirality exception" << std::endl;
              BOOST_FOREACH(const Pose3& pose, cameraPoses) {
                std::cout << " Pose: " << pose << std::endl;
              }
            }
            numFailures++;
            continue;
          }
        }else{ // we read 3D points from file
          if (loadedValues->exists<Point3>((*pfit).first)){ // (*pfit).first) is the key of the landmark
            // point
          }else{
            std::cout << "Trying to read non existing point from file " << std::endl;
          }
        }

        // Add projection factors and pose values
        for (vfit = projectionFactorVector.begin(); vfit != projectionFactorVector.end(); vfit++) { // for each proj factor connected to the landmark
          numProjectionFactorsAdded++;

          if (debug) std::cout << "Adding factor " << std::endl;
          if (debug) (*vfit)->print("Projection Factor");

          graph.push_back( (*vfit) ); // add factor to the graph

          if (!graphValues->exists<Pose3>( (*vfit)->key1()) && loadedValues->exists<Pose3>((*vfit)->key1())) {
            graphValues->insert((*vfit)->key1(), loadedValues->at<Pose3>((*vfit)->key1()));
            cameraPoseKeys.push_back( (*vfit)->key1() ); // add camera poses, if necessary
            // std::cout << "Added camera value " << std::endl;
          }
        }

        // Add landmark value
        if (debug) std::cout << "Adding value " << std::endl;
        graphValues->insert( projectionFactorVector[0]->key2(), point); // add point
        // std::cout << "Added point value " << std::endl;
        landmarkKeys.push_back( projectionFactorVector[0]->key2() );

      }
      if (1||debug) std::cout << " # PROJECTION FACTORS CALCULATED: " << numProjectionFactors;
      if (1||debug) std::cout << " # PROJECTION FACTORS ADDED: " << numProjectionFactorsAdded;
      if (1||debug) std::cout << " # FAILURES: " << numFailures << std::endl;
    }

    const SharedNoiseModel noise_;   ///< noise model used
    ///< (important that the order is the same as the keys that we use to create the factor)
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object

    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    std::vector<Key> cameraPoseKeys;
    std::vector<Key> landmarkKeys;
    ProjectionFactorMap projectionFactors;
    CalibrationMap keyCalibrationMap;
    boost::shared_ptr<Ordering> ordering;
    // orderingMethod: 0 - COLAMD, 1 - landmark first, then COLAMD on poses (constrained ordering)
    int orderingMethod;

    unsigned int totalNumMeasurements;
    unsigned int numLandmarks;
    unsigned int numPoses;

  };

}

#endif /* SMARTPROJECTIONFACTORSCREATOR_H_ */
