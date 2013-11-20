#include <boost/python.hpp>
#include <gtsam/slam/PriorFactor.h>

using namespace boost::python;
using namespace gtsam;

using namespace std;

template< class FACTOR, class VALUE >
void exportPriorFactor(const std::string& name){
  class_< FACTOR >(name.c_str(), init<>())
  .def(init< Key, VALUE&, SharedNoiseModel >())
  ;
}