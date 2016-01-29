/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps BetweenFactor for several values to python
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Pose3.h"

using namespace boost::python;
using namespace gtsam;

using namespace std;

// template<class T>
// void exportBetweenFactor(const std::string& name){
//   class_<T>(name, init<>())
//   .def(init<Key, Key, T, SharedNoiseModel>())
//   ;
// }

#define BETWEENFACTOR(T) \
  class_< BetweenFactor<T>, bases<NonlinearFactor>, boost::shared_ptr< BetweenFactor<T> > >("BetweenFactor"#T) \
  .def(init<Key,Key,T,noiseModel::Base::shared_ptr>()) \
  .def("measured", &BetweenFactor<T>::measured, return_internal_reference<>()) \
;

void exportBetweenFactors()
{
  BETWEENFACTOR(Point2)
  BETWEENFACTOR(Rot2)
  BETWEENFACTOR(Pose2)
  BETWEENFACTOR(Point3)
  BETWEENFACTOR(Rot3)
  BETWEENFACTOR(Pose3)
}
