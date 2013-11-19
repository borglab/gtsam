#include <boost/python.hpp>
#include <boost/cstdint.hpp>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

template<class VALUE> void exportPriorFactor(const std::string& name);
template<class VALUE> void exportBetweenFactor(const std::string& name);

BOOST_PYTHON_MODULE(libgeometry){
  using namespace boost::python;

  typedef gtsam::PriorFactor<gtsam::Point2> Point2PriorFactor;
  typedef gtsam::PriorFactor<gtsam::Pose2> Pose2PriorFactor;

  exportPriorFactor<Point2PriorFactor>("Point2PriorFactor");
  exportPriorFactor<Pose2PriorFactor>("Pose2PriorFactor");

  typedef gtsam::BetweenFactor<gtsam::Pose2> Pose2BetweenFactor;

  exportBetweenFactor<Pose2BetweenFactor>("Pose2BetweenFactor");
}