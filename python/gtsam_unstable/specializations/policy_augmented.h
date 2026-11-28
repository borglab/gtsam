// Authors: Frank Dellaert and Akash Ratheesh.
py::reinterpret_borrow<py::class_<gtsam::PolicyAugmentedProposal, gtsam::GaussianProposal,
    std::shared_ptr<gtsam::PolicyAugmentedProposal>>>(m_.attr("PolicyAugmentedProposal"))
    .def(py::init<const gtsam::Matrix&, size_t, size_t,
                  gtsam::PolicyAugmentedProposal::Generator, size_t>(),
         py::arg("covariance"), py::arg("gaussianSamples"),
         py::arg("policySamples"), py::arg("generator"), py::arg("seed") = 42)
    .def_static(
        "withSeededGenerator",
        [](const gtsam::Matrix& covariance, size_t gaussianSamples,
           size_t policySamples,
           gtsam::PolicyAugmentedProposal::SeededGenerator generator,
           size_t seed) {
          return std::make_shared<gtsam::PolicyAugmentedProposal>(
              covariance, gaussianSamples, policySamples, std::move(generator),
              seed);
        },
        py::arg("covariance"), py::arg("gaussianSamples"),
        py::arg("policySamples"), py::arg("generator"), py::arg("seed") = 42);
