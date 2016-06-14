/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps PriorFactor for several values to python
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/slam/PriorFactor.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/navigation/NavState.h"

using namespace boost::python;
using namespace gtsam;

using namespace std;

// template< class FACTOR, class VALUE >
// void exportPriorFactor(const std::string& name){
//   class_< FACTOR >(name.c_str(), init<>())
//   .def(init< Key, VALUE&, SharedNoiseModel >())
//   ;
// }

#define PRIORFACTOR(VALUE) \
  class_< PriorFactor<VALUE>, bases<NonlinearFactor>, boost::shared_ptr< PriorFactor<VALUE> > >("PriorFactor"#VALUE) \
  .def(init<Key,VALUE,noiseModel::Base::shared_ptr>()) \
  .def("prior", &PriorFactor<VALUE>::prior, return_internal_reference<>()) \
;

void exportPriorFactors()
{
  PRIORFACTOR(Point2)
  PRIORFACTOR(Rot2)
  PRIORFACTOR(Pose2)
  PRIORFACTOR(Point3)
  PRIORFACTOR(Rot3)
  PRIORFACTOR(Pose3)
  PRIORFACTOR(Vector3)
  PRIORFACTOR(NavState)
}
