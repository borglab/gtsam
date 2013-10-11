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

//#include <boost/foreach.hpp>
//#include <boost/assign.hpp>
//#include <boost/assign/std/vector.hpp>
//#include <fstream>
//#include <iostream>
#include <utility>

namespace gtsam {

  typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2> ProjectionFactor;
  typedef FastMap<Key, std::vector<boost::shared_ptr<ProjectionFactor> > > ProjectionFactorMap;
  typedef FastMap<Key, int> OrderingMap;

  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class GenericProjectionFactorsCreator {
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
        ProjectionFactor::shared_ptr projectionFactor(new ProjectionFactor(measurement, noise_, poseKey, landmarkKey, K_));

        // Check if landmark exists in mapping
        ProjectionFactorMap::iterator pfit = projectionFactors.find(landmarkKey);
        if (pfit != projectionFactors.end()) {
          if (debug) fprintf(stderr,"Adding measurement to existing landmark\n");

          // Add projection factor to list of projection factors associated with this landmark
          (*pfit).second.push_back(projectionFactor);

        } else {
          if (debug) fprintf(stderr,"New landmark (%d)\n", pfit != projectionFactors.end());

          // Create a new vector of projection factors
          std::vector<ProjectionFactor::shared_ptr> projectionFactorVector;
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

    void update(NonlinearFactorGraph &graph, gtsam::Values::shared_ptr inputValues, gtsam::Values::shared_ptr outputValues) {
        addTriangulatedLandmarks(graph, inputValues, outputValues);
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
        gtsam::Values::shared_ptr graphValues) {

      bool debug = false;

      std::vector<boost::shared_ptr<ProjectionFactor> > projectionFactorVector;
      std::vector<boost::shared_ptr<ProjectionFactor> >::iterator vfit;
      Point3 point;
      Pose3 cameraPose;

      ProjectionFactorMap::iterator pfit;

      if (debug)  graphValues->print("graphValues \n");
      if (debug) std::cout << " # END VALUES: " << std::endl;

      // Iterate through all landmarks
      if (debug) std::cout << " PROJECTION FACTOR GROUPED: " << projectionFactors.size();
      int numProjectionFactors = 0;
      int numProjectionFactorsAdded = 0;
      int numFailures = 0;
      for (pfit = projectionFactors.begin(); pfit != projectionFactors.end(); pfit++) {

        projectionFactorVector = (*pfit).second;

        std::vector<Pose3> cameraPoses;
        std::vector<Point2> measured;

        // Iterate through projection factors
        for (vfit = projectionFactorVector.begin(); vfit != projectionFactorVector.end(); vfit++) {
          numProjectionFactors++;

          if (debug) std::cout << "ProjectionFactor: " << std::endl;
          if (debug) (*vfit)->print("ProjectionFactor");

          // Iterate through poses
          cameraPoses.push_back( loadedValues->at<Pose3>((*vfit)->key1() ) );
          measured.push_back( (*vfit)->measured() );
        }

        // Triangulate landmark based on set of poses and measurements
        if (debug) std::cout << "Triangulating: " << std::endl;
        try {
          point = triangulatePoint3(cameraPoses, measured, *K_);
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

        // Add projection factors and pose values
        for (vfit = projectionFactorVector.begin(); vfit != projectionFactorVector.end(); vfit++) {
          numProjectionFactorsAdded++;
          if (debug) std::cout << "Adding factor " << std::endl;
          if (debug) (*vfit)->print("Projection Factor");
          graph.push_back( (*vfit) );

          if (!graphValues->exists<Pose3>( (*vfit)->key1()) && loadedValues->exists<Pose3>((*vfit)->key1())) {
            graphValues->insert((*vfit)->key1(), loadedValues->at<Pose3>((*vfit)->key1()));
            cameraPoseKeys.push_back( (*vfit)->key1() );
          }
        }

        // Add landmark value
        if (debug) std::cout << "Adding value " << std::endl;
        graphValues->insert( projectionFactorVector[0]->key2(), point); // add point;
        landmarkKeys.push_back( projectionFactorVector[0]->key2() );

      }
      if (1||debug) std::cout << " # PROJECTION FACTORS CALCULATED: " << numProjectionFactors;
      if (1||debug) std::cout << " # PROJECTION FACTORS ADDED: " << numProjectionFactorsAdded;
      if (1||debug) std::cout << " # FAILURES: " << numFailures;
    }

    const SharedNoiseModel noise_;   ///< noise model used
    ///< (important that the order is the same as the keys that we use to create the factor)
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object

    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    std::vector<Key> cameraPoseKeys;
    std::vector<Key> landmarkKeys;
    ProjectionFactorMap projectionFactors;
    boost::shared_ptr<Ordering> ordering;
    // orderingMethod: 0 - COLAMD, 1 - landmark first, then COLAMD on poses (constrained ordering)
    int orderingMethod;

    unsigned int totalNumMeasurements;
    unsigned int numLandmarks;
    unsigned int numPoses;

  };

}

#endif /* SMARTPROJECTIONFACTORSCREATOR_H_ */
