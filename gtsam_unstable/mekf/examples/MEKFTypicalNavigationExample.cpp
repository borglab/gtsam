#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "MEKFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "MEKFTypicalNavigationExample", "mekf_typical_navigation.csv",
      "Typical-navigation", imu_scenarios::typicalNavigation,
      mekf::examples::runScenario);
}
