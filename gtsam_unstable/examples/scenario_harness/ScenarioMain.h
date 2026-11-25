/**
 * @file  ScenarioMain.h
 * @brief The body of every trajectory-example executable.
 *
 * Each filter has one example executable, `<Prefix>ScenarioExample`, that
 * runs the trajectory named by `--scenario`. This file holds the scenario
 * table and the shared main; the executables only name their filter.
 */
#pragma once

#include <gtsam_unstable/examples/scenario_harness/IMUScenarios.h>
#include <gtsam_unstable/examples/scenario_harness/ScenarioHarness.h>

#include <array>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace imu_scenarios {

/// A trajectory selectable with `--scenario`.
struct ScenarioEntry {
  const char* name;        ///< CLI name and run-summary label.
  const char* csv_suffix;  ///< Default output is `<stem>_<suffix>.csv`.
  std::unique_ptr<gtsam::Scenario> (*make)();
};

/// Moves a concrete scenario onto the heap behind the common base.
template <class ConcreteScenario>
std::unique_ptr<gtsam::Scenario> own(ConcreteScenario scenario) {
  return std::make_unique<ConcreteScenario>(std::move(scenario));
}

/// Every trajectory the examples can run.
inline const std::array<ScenarioEntry, 3>& scenarioTable() {
  static const std::array<ScenarioEntry, 3> table{{
      {"Vertical", "vertical", [] { return own(vertical()); }},
      {"TypicalNavigation", "typical_navigation",
       [] { return own(typicalNavigation()); }},
      {"WaypointSpline", "waypoint_spline",
       [] { return own(waypointSpline()); }},
  }};
  return table;
}

/// The table entry named `name`; throws, listing the valid names, otherwise.
inline const ScenarioEntry& findScenario(const std::string& name) {
  std::string valid;
  for (const ScenarioEntry& entry : scenarioTable()) {
    if (name == entry.name) return entry;
    valid += valid.empty() ? entry.name : std::string(", ") + entry.name;
  }
  throw std::runtime_error("--scenario requires one of: " + valid);
}

/**
 * Parse the shared CLI, build the scenario named by `--scenario`, and run the
 * filter over it. Without `--output`, the CSV goes to
 * `<output_stem>_<csv_suffix>.csv`.
 *
 * @param exe_name     executable name, used for the error message only.
 * @param output_stem  filter stem of the default CSV name, e.g. "tg_eqf".
 * @param run          the filter's examples::runScenario.
 * @return process exit status: 0 on success, 2 on a bad CLI or a failed run.
 */
template <class RunScenario>
inline int exampleMain(int argc, char* argv[], const char* exe_name,
                       const char* output_stem, RunScenario run) {
  try {
    RunOptions opts = parseRunOptions(argc, argv, "");
    const ScenarioEntry& entry = findScenario(opts.scenario);
    if (opts.output_path.empty()) {
      opts.output_path =
          std::string(output_stem) + "_" + entry.csv_suffix + ".csv";
    }
    const std::unique_ptr<gtsam::Scenario> scenario = entry.make();
    run(*scenario, opts, entry.name);
  } catch (const std::exception& e) {
    std::cerr << exe_name << " failed: " << e.what() << '\n';
    return 2;
  }
  return 0;
}

}  // namespace imu_scenarios
