#include <boost/python.hpp>
#include <gtsam/slam/PriorFactor.h>

using namespace boost::python;
using namespace gtsam;

using namespace std;

template<class VALUE>
void exposePriorFactor(const std::string& name){
  class_<VALUE>(name, init<>())
  .def(init<Key, VALUE, SharedNoiseModel>())
  ;
}