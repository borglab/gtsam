/**
 * @file MEKFSamoaSurveyExample.cpp
 * @brief MEKF on a waypoint spline through a real AUV survey trajectory;
 *        exercises the translational channels and the position/DVL aiding
 *        path against a realistic (not hand-designed) path shape.
 */
#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "MEKFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "MEKFSamoaSurveyExample", "mekf_samoa_survey.csv",
      "SamoaSurvey", [] { return imu_scenarios::samoaSurvey(); },
      mekf::examples::runScenario);
}
