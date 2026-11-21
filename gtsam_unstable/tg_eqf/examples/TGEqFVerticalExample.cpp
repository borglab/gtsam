/**
 * @file TGEqFVerticalExample.cpp
 * @brief TG-EqF vertical (gravity-axis) trajectory; accel-bias vs gravity.
 */
#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "TGEqFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "TGEqFVerticalExample", "tg_eqf_vertical.csv", "Vertical",
      imu_scenarios::vertical, gtsam::tgeqf::examples::runScenario);
}
