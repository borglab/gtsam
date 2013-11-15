#include <boost/python.hpp>
#include "gtsam/base/Value.h"

using namespace boost::python;
using namespace gtsam;

// Virtual class, no init
void exportValue(){
  class_<Value>("Value", no_init);
} 