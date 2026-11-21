/**
 * @file MEKFWaypointSplineExample.cpp
 * @brief MEKF on a cubic spline through predefined 3-D waypoints; exercises the
 *        translational channels and the position/DVL aiding path.
 */
#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "MEKFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "MEKFWaypointSplineExample", "mekf_waypoint_spline.csv",
      "WaypointSpline", [] { return imu_scenarios::waypointSpline(); },
      mekf::examples::runScenario);
}
