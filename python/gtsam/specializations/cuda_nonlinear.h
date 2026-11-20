// GNC templates live in gtsam in C++; also expose their CUDA instantiations
// alongside the inner optimizers without registering the native types twice.
m_.attr("cuda").attr("GncSparseLMParams") = m_.attr("GncCudaSparseLMParams");
m_.attr("cuda").attr("GncSparseLMOptimizer") = m_.attr("GncCudaSparseLMOptimizer");

// Both run OpenMP-parallel linearize() over a CustomFactor's Python
// callback; no wrap binding releases the GIL by default, so an OpenMP
// worker re-acquiring it deadlocks (reproduced directly). Hand-bound here
// with call_guard<gil_scoped_release>() since .i syntax can't express it.
{
  auto batchedLm = py::reinterpret_borrow<py::class_<
      gtsam::cuda::BatchedDenseLevenbergMarquardtOptimizer,
      std::shared_ptr<gtsam::cuda::BatchedDenseLevenbergMarquardtOptimizer>>>(
      m_.attr("cuda").attr("BatchedDenseLevenbergMarquardtOptimizer"));
  batchedLm.def(
      "optimize",
      static_cast<const std::vector<gtsam::Values>& (
          gtsam::cuda::BatchedDenseLevenbergMarquardtOptimizer::*)()>(
          &gtsam::cuda::BatchedDenseLevenbergMarquardtOptimizer::optimize),
      py::call_guard<py::gil_scoped_release>(),
      py::return_value_policy::reference_internal);

  auto batchedGradients = py::reinterpret_borrow<py::class_<
      gtsam::cuda::BatchedDeviceGradients,
      std::shared_ptr<gtsam::cuda::BatchedDeviceGradients>>>(
      m_.attr("cuda").attr("BatchedDeviceGradients"));
  batchedGradients.def(
      py::init<const std::vector<gtsam::NonlinearFactorGraph>&,
               const std::vector<gtsam::Values>&,
               const gtsam::cuda::BatchedDenseSolverOptions&>(),
      py::call_guard<py::gil_scoped_release>(),
      gtwrap::internal::py_arg<const std::vector<gtsam::NonlinearFactorGraph>&>(
          "graphs"),
      gtwrap::internal::py_arg<const std::vector<gtsam::Values>&>("solutions"),
      gtwrap::internal::py_arg<const gtsam::cuda::BatchedDenseSolverOptions&>(
          "options") = gtsam::cuda::BatchedDenseSolverOptions());
}
