#include <boost/python.hpp>
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace boost::python;
using namespace gtsam;


void exportNonlinearFactorGraph(){

  typedef boost::shared_ptr<NonlinearFactor> shared_factor;

  void (NonlinearFactorGraph::*push_back1)(const shared_factor& factor) = &NonlinearFactorGraph::push_back;

  class_<NonlinearFactorGraph>("NonlinearFactorGraph", init<>())
  .def("push_back", push_back1)
  ;
}