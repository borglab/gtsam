// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file mppi_variants.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
{
  using namespace gtsam;
  py::reinterpret_borrow<py::class_<DDPTracking, std::shared_ptr<DDPTracking>>>(
      m_.attr("DDPTracking"))
      .def("optimize", &DDPTracking::optimize, py::arg("initial"),
           py::arg("targetStates"), py::arg("targetControls"),
           py::arg("transition"), py::arg("iterations"),
           py::arg("lower") = Vector(), py::arg("upper") = Vector(),
           py::call_guard<py::gil_scoped_release>());
  auto binding = py::reinterpret_borrow<
      py::class_<MPPITracking, std::shared_ptr<MPPITracking>>>(
      m_.attr("MPPITracking"));
  auto bindTracking = [&](auto* prototype) {
    using State = std::remove_pointer_t<decltype(prototype)>;
    using Dynamics = std::function<typename traits<State>::TangentVector(
        const State&, const Vector&)>;
    using Feedback = std::function<Vector(size_t, const State&, const State&,
                                          const Vector&)>;
    binding
        .def("colored", &MPPITracking::colored<State, Vector, Dynamics>,
             py::arg("planner"), py::arg("graph"), py::arg("initial"),
             py::arg("dynamics"), py::arg("exponent"), py::arg("iterations"),
             py::arg("tsallisCutoff") = 0.0, py::arg("tsallisShape") = 2.0,
             py::call_guard<py::gil_scoped_release>())
        .def("tube", &MPPITracking::tube<State, Vector, Dynamics, Feedback>,
             py::arg("planner"), py::arg("graph"), py::arg("nominal"),
             py::arg("actual"), py::arg("dynamics"), py::arg("feedback"),
             py::arg("threshold"), py::arg("iterations"),
             py::call_guard<py::gil_scoped_release>())
        .def("robust", &MPPITracking::robust<State, Vector, Dynamics, Feedback>,
             py::arg("planner"), py::arg("graph"), py::arg("previousNominal"),
             py::arg("actual"), py::arg("dynamics"), py::arg("feedback"),
             py::arg("threshold"), py::arg("candidates"), py::arg("stride"),
             py::arg("iterations"), py::arg("feedbackPrecision"),
             py::call_guard<py::gil_scoped_release>())
        .def("primitives", &MPPITracking::primitives<State, Vector, Dynamics>,
             py::arg("planner"), py::arg("graph"), py::arg("initial"),
             py::arg("previousMPPI"), py::arg("dynamics"), py::arg("segments"),
             py::arg("exponent"), py::arg("hysteresis"),
             py::arg("primitiveIterations"), py::arg("mppiIterations"),
             py::arg("stopThreshold"), py::arg("stoppingControls"),
             py::arg("tsallisCutoff") = 0.0, py::arg("tsallisShape") = 2.0,
             py::arg("mixture") = MotionPrimitiveOptions{},
             py::call_guard<py::gil_scoped_release>());
    auto bindProposal = [&](auto* prototype) {
      using Proposal = std::remove_pointer_t<decltype(prototype)>;
      binding
          .def("tube",
               &MPPITracking::tube<State, Vector, Dynamics, Feedback, Proposal>,
               py::arg("planner"), py::arg("graph"), py::arg("nominal"),
               py::arg("actual"), py::arg("dynamics"), py::arg("feedback"),
               py::arg("proposal"), py::arg("threshold"), py::arg("iterations"),
               py::call_guard<py::gil_scoped_release>())
          .def("robust",
               &MPPITracking::robust<State, Vector, Dynamics, Feedback,
                                     Proposal>,
               py::arg("planner"), py::arg("graph"), py::arg("previousNominal"),
               py::arg("actual"), py::arg("dynamics"), py::arg("feedback"),
               py::arg("proposal"), py::arg("threshold"), py::arg("candidates"),
               py::arg("stride"), py::arg("iterations"),
               py::arg("feedbackPrecision"),
               py::call_guard<py::gil_scoped_release>());
    };
    // Colored and normal-lognormal proposals use the Gaussian base binding.
    bindProposal(static_cast<GaussianProposal*>(nullptr));
    bindProposal(static_cast<SmoothControlProposal*>(nullptr));
    bindProposal(static_cast<PiecewiseLinearProposal*>(nullptr));
  };
  bindTracking(static_cast<Vector*>(nullptr));
  bindTracking(static_cast<Pose2*>(nullptr));
}
