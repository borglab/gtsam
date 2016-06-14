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
#include "gtsam/navigation/GPSFactor.h"

#include "python/handwritten/common.h"

using namespace boost::python;
using namespace gtsam;

typedef gtsam::OptionalJacobian<3, 9> OptionalJacobian39;
typedef gtsam::OptionalJacobian<9, 6> OptionalJacobian96;
typedef gtsam::OptionalJacobian<9, 9> OptionalJacobian9;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(print_overloads, print, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(attitude_overloads, attitude, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(position_overloads, position, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(velocity_overloads, velocity, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(equals_overloads, PreintegratedImuMeasurements::equals, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(predict_overloads, PreintegrationBase::predict, 2, 4)

void exportImuFactor() {
  class_<OptionalJacobian39>("OptionalJacobian39", init<>());
  class_<OptionalJacobian96>("OptionalJacobian96", init<>());
  class_<OptionalJacobian9>("OptionalJacobian9", init<>());

  class_<NavState>("NavState", init<>())
      .def(init<const Rot3&, const Point3&, const Velocity3&>())
      // TODO(frank): overload with jacobians
      .def("print", &NavState::print, print_overloads())
      .def("attitude", &NavState::attitude,
           attitude_overloads()[return_value_policy<copy_const_reference>()])
      .def("position", &NavState::position,
           position_overloads()[return_value_policy<copy_const_reference>()])
      .def("velocity", &NavState::velocity,
           velocity_overloads()[return_value_policy<copy_const_reference>()])
      .def(repr(self))
      .def("pose", &NavState::pose);

  class_<imuBias::ConstantBias>("ConstantBias", init<>())
      .def(init<const Vector3&, const Vector3&>())
      .def(repr(self));

  class_<PreintegrationParams, boost::shared_ptr<PreintegrationParams>>(
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

  // NOTE(frank): https://mail.python.org/pipermail/cplusplus-sig/2016-January/017362.html
  REGISTER_SHARED_PTR_TO_PYTHON(PreintegrationParams);

  class_<PreintegrationType>(
#ifdef GTSAM_TANGENT_PREINTEGRATION
      "TangentPreintegration",
#else
      "ManifoldPreintegration",
#endif
      init<const boost::shared_ptr<PreintegrationParams>&, const imuBias::ConstantBias&>())
      .def("predict", &PreintegrationType::predict, predict_overloads())
      .def("computeError", &PreintegrationType::computeError)
      .def("resetIntegration", &PreintegrationType::resetIntegration)
      .def("deltaTij", &PreintegrationType::deltaTij);

  class_<PreintegratedImuMeasurements, bases<PreintegrationType>>(
      "PreintegratedImuMeasurements",
      init<const boost::shared_ptr<PreintegrationParams>&, const imuBias::ConstantBias&>())
      .def(repr(self))
      .def("equals", &PreintegratedImuMeasurements::equals, equals_overloads(args("other", "tol")))
      .def("integrateMeasurement", &PreintegratedImuMeasurements::integrateMeasurement)
      .def("integrateMeasurements", &PreintegratedImuMeasurements::integrateMeasurements)
      .def("preintMeasCov", &PreintegratedImuMeasurements::preintMeasCov);

  class_<ImuFactor, bases<NonlinearFactor>, boost::shared_ptr<ImuFactor>>("ImuFactor")
      .def("error", &ImuFactor::error)
      .def(init<Key, Key, Key, Key, Key, const PreintegratedImuMeasurements&>())
      .def(repr(self));
  REGISTER_SHARED_PTR_TO_PYTHON(ImuFactor);

  class_<ImuFactor2, bases<NonlinearFactor>, boost::shared_ptr<ImuFactor2>>("ImuFactor2")
      .def("error", &ImuFactor2::error)
      .def(init<Key, Key, Key, const PreintegratedImuMeasurements&>())
      .def(repr(self));
  REGISTER_SHARED_PTR_TO_PYTHON(ImuFactor2);

  class_<GPSFactor, bases<NonlinearFactor>, boost::shared_ptr<GPSFactor>>("GPSFactor")
      .def("error", &GPSFactor::error)
      .def(init<Key, const Point3&, noiseModel::Base::shared_ptr>());
  REGISTER_SHARED_PTR_TO_PYTHON(GPSFactor);

  class_<GPSFactor2, bases<NonlinearFactor>, boost::shared_ptr<GPSFactor2>>("GPSFactor2")
      .def("error", &GPSFactor2::error)
      .def(init<Key, const Point3&, noiseModel::Base::shared_ptr>());
  REGISTER_SHARED_PTR_TO_PYTHON(GPSFactor2);
}
