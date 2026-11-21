#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioMain.h>

#include "TFGInEKFScenarioExample.h"

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(
      argc, argv, "TFGInEKFSamoaSurveyExample",
      "tfg_inekf_samoa_survey.csv", "SamoaSurvey",
      [] { return imu_scenarios::samoaSurvey(); },
      tfg::examples::runScenario);
}
