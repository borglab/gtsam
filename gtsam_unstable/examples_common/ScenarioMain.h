/**
 * @file  ScenarioMain.h
 * @brief The body of every trajectory-example executable.
 *
 * One executable per (filter, scenario) pair is kept deliberately: the analysis
 * campaign runner locates them by name, `<Prefix><Scenario>Example`. Each of
 * those mains only names a scenario and an output file; everything else is
 * here.
 */
#pragma once

#include <gtsam_unstable/examples_common/ScenarioHarness.h>

#include <exception>
#include <iostream>

namespace imu_scenarios {

/// Parse the shared CLI, build the scenario, run the filter over it.
/// @param exe_name        executable name, used for the error message only.
/// @param default_output  CSV path when --output is not given.
/// @param scenario_name   label printed in the run summary.
/// @param make_scenario   nullary callable returning the gtsam::Scenario.
/// @param run             the filter's examples::runScenario.
/// @return process exit status: 0 on success, 2 if the run threw.
template <class MakeScenario, class RunScenario>
inline int exampleMain(int argc, char* argv[], const char* exe_name,
                       const char* default_output, const char* scenario_name,
                       MakeScenario make_scenario, RunScenario run) {
  try {
    const RunOptions opts = parseRunOptions(argc, argv, default_output);
    const auto scenario = make_scenario();
    run(scenario, opts, scenario_name);
  } catch (const std::exception& e) {
    std::cerr << exe_name << " failed: " << e.what() << '\n';
    return 2;
  }
  return 0;
}

}  // namespace imu_scenarios
