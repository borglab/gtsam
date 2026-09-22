/* Please refer to:
 * https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
 * These are required to save one copy operation on Python calls.
 *
 * NOTES
 * =================
 *
 * `py::bind_vector` and similar machinery gives the std container a Python-like
 * interface, but without the `<pybind11/stl.h>` copying mechanism. Combined
 * with `PYBIND11_MAKE_OPAQUE` this allows the types to be modified with Python,
 * and saves one copy operation.
 */

// Register before NoiseModelFactor::unwhitenedError so pybind11 can use the
// Python type name in that method's signature. CustomFactor is bound later.
py::bind_vector<std::vector<gtsam::Matrix>>(
    m_, "JacobianVector", py::module_local(false));

{
  using PythonAffectedKeys =
      std::map<gtsam::FactorIndex, std::set<gtsam::Key>>;

  auto updateParams = py::reinterpret_borrow<
      py::class_<gtsam::ISAM2UpdateParams,
                 std::shared_ptr<gtsam::ISAM2UpdateParams>>>(
      m_.attr("ISAM2UpdateParams"));
  updateParams.def_property(
      "newAffectedKeys",
      [](const gtsam::ISAM2UpdateParams& params)
          -> std::optional<PythonAffectedKeys> {
        if (!params.newAffectedKeys) {
          return std::nullopt;
        }

        PythonAffectedKeys result;
        for (const auto& [factorIndex, keys] : *params.newAffectedKeys) {
          result.emplace(factorIndex,
                         std::set<gtsam::Key>(keys.begin(), keys.end()));
        }
        return result;
      },
      [](gtsam::ISAM2UpdateParams& params,
         const std::optional<PythonAffectedKeys>& affectedKeys) {
        if (!affectedKeys) {
          params.newAffectedKeys.reset();
          return;
        }

        gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> result;
        for (const auto& [factorIndex, keys] : *affectedKeys) {
          result.emplace(factorIndex, gtsam::KeySet(keys));
        }
        params.newAffectedKeys = std::move(result);
      });
}
