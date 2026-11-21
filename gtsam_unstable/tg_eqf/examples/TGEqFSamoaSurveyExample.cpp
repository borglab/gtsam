/**
 * @file TGEqFSamoaSurveyExample.cpp
 * @brief TG-EqF on a waypoint spline through a real AUV survey trajectory;
 *        exercises the translational channels and the position/DVL aiding
 *        path against a realistic (not hand-designed) path shape.
 */
#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "TGEqFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "TGEqFSamoaSurveyExample", "tg_eqf_samoa_survey.csv",
      "SamoaSurvey", [] { return imu_scenarios::samoaSurvey(); },
      gtsam::tgeqf::examples::runScenario);
}
