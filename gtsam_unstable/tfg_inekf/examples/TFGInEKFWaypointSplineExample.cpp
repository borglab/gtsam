#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "TFGInEKFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "TFGInEKFWaypointSplineExample",
      "tfg_inekf_waypoint_spline.csv", "WaypointSpline",
      [] { return imu_scenarios::waypointSpline(); },
      tfg::examples::runScenario);
}
