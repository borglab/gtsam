#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "TFGInEKFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "TFGInEKFVerticalExample", "tfg_inekf_vertical.csv",
      "Vertical", imu_scenarios::vertical, tfg::examples::runScenario);
}
