/**
 * @file TGEqFTypicalNavigationExample.cpp
 * @brief TG-EqF typical ground-navigation route: cruise with occasional turns.
 */
#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "TGEqFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "TGEqFTypicalNavigationExample",
      "tg_eqf_typical_navigation.csv", "Typical-navigation",
      imu_scenarios::typicalNavigation, gtsam::tgeqf::examples::runScenario);
}
