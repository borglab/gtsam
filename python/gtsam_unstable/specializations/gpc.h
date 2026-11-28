// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
// Authors: Frank Dellaert and Akash Ratheesh.

{
  using namespace gtsam;
  auto model = py::reinterpret_borrow<
      py::class_<GenerativePredictiveControl,
                 std::shared_ptr<GenerativePredictiveControl>>>(
      m_.attr("GenerativePredictiveControl"));
  model.def("trainCycle", &GenerativePredictiveControl::trainCycle,
            py::arg("initialStates"), py::arg("params"), py::arg("observe"),
            py::arg("advance"), py::arg("scoreBatch"), py::arg("weights"));
}
