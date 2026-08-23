#include <CppUnitLite/TestHarness.h>

#include "../timing/sfm_ba/GncOutlierSampling.h"

#include <set>
#include <utility>
#include <vector>

using gtsam::timing::SelectConstrainedOutlierMeasurements;

/* ************************************************************************* */
TEST(GncOutlierSampling, ZeroRequestedSelectsNone) {
  const auto selected =
      SelectConstrainedOutlierMeasurements(std::vector<size_t>{3, 5}, 0, 42);
  CHECK(selected.empty());
}

/* ************************************************************************* */
TEST(GncOutlierSampling, IsDeterministicAndRespectsTrackCapacity) {
  const std::vector<size_t> trackSizes{2, 3, 5};
  const auto first =
      SelectConstrainedOutlierMeasurements(trackSizes, 10, 42);
  const auto second =
      SelectConstrainedOutlierMeasurements(trackSizes, 10, 42);

  CHECK(first == second);
  LONGS_EQUAL(4, first.size());

  std::vector<size_t> counts(trackSizes.size(), 0);
  for (const auto& selected : first) {
    ++counts[selected.first];
  }
  LONGS_EQUAL(0, counts[0]);
  LONGS_EQUAL(1, counts[1]);
  LONGS_EQUAL(3, counts[2]);
}

/* ************************************************************************* */
TEST(GncOutlierSampling, EveryMeasurementPositionIsEligible) {
  std::set<size_t> reached;
  for (unsigned int seed = 0; seed < 256; ++seed) {
    const auto selected =
        SelectConstrainedOutlierMeasurements(std::vector<size_t>{5}, 1, seed);
    LONGS_EQUAL(1, selected.size());
    reached.insert(selected.front().second);
  }
  LONGS_EQUAL(5, reached.size());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
