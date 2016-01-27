/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps ConstantTwistScenario class to python
 * @author Frank Dellaert
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/navigation/ImuFactor.h"

using namespace boost::python;
using namespace gtsam;

void exportImuFactor() {
  class_<NavState>("NavState", init<>())
      // TODO(frank): overload with jacobians
      //      .def("attitude", &NavState::attitude)
      //      .def("position", &NavState::position)
      //      .def("velocity", &NavState::velocity)
      .def(repr(self))
      .def("pose", &NavState::pose);

  class_<imuBias::ConstantBias>("ConstantBias", init<>())
      .def(init<const Vector3&, const Vector3&>())
      .def(repr(self));

  class_<PreintegrationParams, boost::shared_ptr<PreintegrationParams> >(
      "PreintegrationParams", init<const Vector3&>())
      .def_readwrite("gyroscopeCovariance",
                     &PreintegrationParams::gyroscopeCovariance)
      .def_readwrite("omegaCoriolis", &PreintegrationParams::omegaCoriolis)
      .def_readwrite("body_P_sensor", &PreintegrationParams::body_P_sensor)
      .def_readwrite("accelerometerCovariance",
                     &PreintegrationParams::accelerometerCovariance)
      .def_readwrite("integrationCovariance",
                     &PreintegrationParams::integrationCovariance)
      .def_readwrite("use2ndOrderCoriolis",
                     &PreintegrationParams::use2ndOrderCoriolis)
      .def_readwrite("n_gravity", &PreintegrationParams::n_gravity)

      .def("MakeSharedD", &PreintegrationParams::MakeSharedD)
      .staticmethod("MakeSharedD")
      .def("MakeSharedU", &PreintegrationParams::MakeSharedU)
      .staticmethod("MakeSharedU");

  class_<PreintegratedImuMeasurements>(
      "PreintegratedImuMeasurements",
      init<const boost::shared_ptr<PreintegrationParams>&,
           const imuBias::ConstantBias&>())
      .def(repr(self))
      .def("resetIntegration", &PreintegratedImuMeasurements::resetIntegration)
      .def("integrateMeasurement",
           &PreintegratedImuMeasurements::integrateMeasurement)
      .def("preintMeasCov", &PreintegratedImuMeasurements::preintMeasCov);

  // NOTE(frank): Abstract classes need boost::noncopyable
  class_<ImuFactor, boost::noncopyable>("ImuFactor", no_init);
}
