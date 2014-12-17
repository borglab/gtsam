#include <boost/python.hpp>
#include <boost/cstdint.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace boost::python;
using namespace gtsam;
using namespace std;

// Geometery
void exportPoint2();
void exportRot2();
void exportPose2();

// Linear
void exportNoiseModels();

// Nonlinear
void exportValues();
void exportNonlinearFactorGraph();
void exportLevenbergMarquardtOptimizer();

// Slam
template< class FACTOR, class VALUE >
void exportPriorFactor(const std::string& name){
  class_< FACTOR >(name.c_str(), init<>())
  .def(init< Key, VALUE&, SharedNoiseModel >())
  ;
}

template<class FACTOR, class VALUE>
void exportBetweenFactor(const std::string& name){
  class_<FACTOR>(name.c_str(), init<>())
  .def(init<Key, Key, VALUE&, SharedNoiseModel>())
  ;
}

typedef gtsam::PriorFactor<gtsam::Point2> Point2PriorFactor;
typedef gtsam::PriorFactor<gtsam::Pose2> Pose2PriorFactor;
typedef gtsam::BetweenFactor<gtsam::Pose2> Pose2BetweenFactor;

//-----------------------------------//

BOOST_PYTHON_MODULE(libgtsam){
  using namespace boost::python;
  exportPoint2();
  exportRot2();
  exportPose2();

  exportNoiseModels();

  exportValues();
  exportNonlinearFactorGraph();
  exportLevenbergMarquardtOptimizer();

  exportPriorFactor< Point2PriorFactor, Point2 >("Point2PriorFactor");
  exportPriorFactor< Pose2PriorFactor, Pose2 >("Pose2PriorFactor");
  exportBetweenFactor< Pose2BetweenFactor, Pose2 >("Pose2BetweenFactor");
}