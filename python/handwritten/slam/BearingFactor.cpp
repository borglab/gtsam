#include <boost/python.hpp>
#include <gtsam/slam/BearingFactor.h>

using namespace boost::python;
using namespace gtsam;

using namespace std;

template<class VALUE>
void exportBearingFactor(const std::string& name){
  class_<VALUE>(name, init<>())
  ;
}