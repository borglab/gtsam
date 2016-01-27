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

#include "gtsam/navigation/ScenarioRunner.h"

using namespace boost::python;
using namespace gtsam;

// Create const Scenario pointer from ConstantTwistScenario
static const Scenario* ScenarioPointer(const ConstantTwistScenario& scenario) {
  return static_cast<const Scenario*>(&scenario);
}

void exportScenario() {
  // NOTE(frank): Abstract classes need boost::noncopyable
  class_<Scenario, boost::noncopyable>("Scenario", no_init);

  // TODO(frank): figure out how to do inheritance
  class_<ConstantTwistScenario>("ConstantTwistScenario",
                                init<const Vector3&, const Vector3&>())
      .def("pose", &Scenario::pose)
      .def("omega_b", &Scenario::omega_b)
      .def("velocity_n", &Scenario::velocity_n)
      .def("acceleration_n", &Scenario::acceleration_n)
      .def("rotation", &Scenario::rotation)
      .def("velocity_b", &Scenario::velocity_b)
      .def("acceleration_b", &Scenario::acceleration_b);

  // NOTE(frank): https://wiki.python.org/moin/boost.python/CallPolicy
  def("ScenarioPointer", &ScenarioPointer,
      return_value_policy<reference_existing_object>());

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
           const imuBias::ConstantBias&>()).def(repr(self));

  class_<NavState>("NavState", init<>())
      // TODO(frank): overload with jacobians
      //      .def("attitude", &NavState::attitude)
      //      .def("position", &NavState::position)
      //      .def("velocity", &NavState::velocity)
      .def(repr(self))
      .def("pose", &NavState::pose);

  class_<imuBias::ConstantBias>("ConstantBias", init<>());

  class_<ScenarioRunner>(
      "ScenarioRunner",
      init<const Scenario*, const boost::shared_ptr<PreintegrationParams>&,
           double>())
      .def("actualSpecificForce", &ScenarioRunner::actualSpecificForce)
      .def("measuredAngularVelocity", &ScenarioRunner::measuredAngularVelocity)
      .def("measuredSpecificForce", &ScenarioRunner::measuredSpecificForce)
      .def("imuSampleTime", &ScenarioRunner::imuSampleTime,
           return_value_policy<copy_const_reference>())
      .def("integrate", &ScenarioRunner::integrate)
      .def("predict", &ScenarioRunner::predict)
      .def("estimateCovariance", &ScenarioRunner::estimateCovariance);
}
