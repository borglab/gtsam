#include <boost/python.hpp>
#include <gtsam/slam/BetweenFactor.h>

using namespace boost::python;
using namespace gtsam;

using namespace std;

template<class VALUE>
void exportBetweenFactor(const std::string& name){
  class_<VALUE>(name, init<>())
  .def(init<Key, Key, VALUE, SharedNoiseModel>())
  ;
}