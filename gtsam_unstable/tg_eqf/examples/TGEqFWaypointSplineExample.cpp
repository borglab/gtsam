/**
 * @file TGEqFWaypointSplineExample.cpp
 * @brief TG-EqF on a cubic spline through predefined 3-D waypoints; exercises
 *        the translational channels and the position/DVL aiding path.
 */
#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "TGEqFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "TGEqFWaypointSplineExample", "tg_eqf_waypoint_spline.csv",
      "WaypointSpline", [] { return imu_scenarios::waypointSpline(); },
      gtsam::tgeqf::examples::runScenario);
}
