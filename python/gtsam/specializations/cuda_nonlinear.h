// GNC templates live in gtsam in C++; also expose their CUDA instantiations
// alongside the inner optimizers without registering the native types twice.
m_.attr("cuda").attr("GncSparseLMParams") = m_.attr("GncCudaSparseLMParams");
m_.attr("cuda").attr("GncSparseLMOptimizer") = m_.attr("GncCudaSparseLMOptimizer");

// BatchedDenseLevenbergMarquardtOptimizer::optimize() and
// BatchedDeviceGradients's constructor each run an OpenMP parallel region
// that calls graphs[i].linearize(...) across the batch's independent
// items. linearize() calls every factor's own linearize()/evaluateError,
// which for a CustomFactor means calling back into an arbitrary Python
// callback. None of this module's wrap-generated .def() bindings release
// the GIL, so the calling thread holds it across the whole call; an
// OpenMP worker thread that then tries to re-acquire the GIL to run a
// Python callback blocks forever waiting for a release that only happens
// after the (already-blocked) OpenMP barrier -- a real deadlock,
// reproduced directly with a Python gtsam.CustomFactor, not just reasoned
// about. wrap's .i syntax has no per-method call_guard annotation, so
// these two are deliberately left out of cuda_nonlinear.i and hand-bound
// here instead with py::call_guard<py::gil_scoped_release>(): releasing
// the GIL before entering the C++ call lets pybind11's own std::function
// trampoline safely (if serially, since Python itself still only runs one
// thread at a time) re-acquire it from whichever OpenMP worker thread
// actually needs to run a Python callback, while the rest of each
// worker's (non-Python) work still runs truly in parallel. A pure C++
// caller never goes through either of these bindings at all, so neither
// method's own correctness depends on this -- it is purely what makes the
// *Python* entry point safe.
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
