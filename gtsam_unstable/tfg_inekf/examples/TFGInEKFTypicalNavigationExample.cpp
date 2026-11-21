#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "TFGInEKFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "TFGInEKFTypicalNavigationExample",
      "tfg_inekf_typical_navigation.csv", "Typical-navigation",
      imu_scenarios::typicalNavigation, tfg::examples::runScenario);
}
