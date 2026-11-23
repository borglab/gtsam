// Authors: Frank Dellaert and Akash Ratheesh.
// Typed initialization preserves the fixed-size vectors used by the dynamics
// factors.
{
  const auto bindInitial = [&](auto* tag, const char* name) {
    using Controller = std::remove_pointer_t<decltype(tag)>;
    auto cls = py::reinterpret_borrow<
        py::class_<Controller, std::shared_ptr<Controller>>>(m_.attr(name));
    cls.def("initialValues", &Controller::initialValues, py::arg("graph"),
            py::arg("controlKeys"), py::arg("start"), py::arg("controls"));
  };
  bindInitial(
      static_cast<
          gtsam::VanillaMPPIController<gtsam::PointMassDynamicsFactor>*>(
          nullptr),
      "VanillaMPPIControllerPointMassDynamicsFactor");
  bindInitial(
      static_cast<gtsam::VanillaMPPIController<gtsam::UnicycleDynamicsFactor>*>(
          nullptr),
      "VanillaMPPIControllerUnicycleDynamicsFactor");
}
