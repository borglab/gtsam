#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <CppUnitLite/TestHarness.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

namespace {

constexpr Key kPoseKey = 10;
constexpr Key kPointKey = 20;

class ReversedPointPoseFactor : public NoiseModelFactorN<Point2, Pose2> {
 public:
  using Base = NoiseModelFactorN<Point2, Pose2>;

  ReversedPointPoseFactor(Key pointKey, Key poseKey)
      : Base(noiseModel::Unit::Create(2), pointKey, poseKey) {}

  Vector evaluateError(const Point2& point, const Pose2& pose,
                       OptionalMatrixType Hpoint = OptionalNone,
                       OptionalMatrixType Hpose = OptionalNone) const override {
    if (Hpoint) {
      *Hpoint = Matrix::Identity(2, 2);
    }
    if (Hpose) {
      *Hpose = Matrix::Zero(2, 3);
      Hpose->block<2, 2>(0, 0) = -Matrix::Identity(2, 2);
    }
    return point - pose.translation();
  }
};

class StructuralFactor : public NonlinearFactor {
 public:
  StructuralFactor(const KeyVector& keys, size_t rowCount,
                   bool isSendable = true)
      : NonlinearFactor(keys),
        rowCount_(rowCount),
        isSendable_(isSendable) {}

  size_t dim() const override { return rowCount_; }

  std::shared_ptr<GaussianFactor> linearize(const Values&) const override {
    return {};
  }

  bool sendable() const override { return isSendable_; }

 private:
  size_t rowCount_;
  bool isSendable_;
};

Values makeValues() {
  Values values;
  values.insert(kPoseKey, Pose2());
  values.insert(kPointKey, Point2(4.0, 5.0));
  return values;
}

NonlinearFactorGraph makeGraph() {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  graph.emplace_shared<ReversedPointPoseFactor>(kPointKey, kPoseKey);
  return graph;
}

}  // namespace

TEST(SparseJacobianColumnLayout, UsesNaturalKeyOrderAndConvertsDeltas) {
  const Values values = makeValues();
  const SparseJacobianColumnLayout layout(values);

  EXPECT_LONGS_EQUAL(5, layout.totalColumns());
  EXPECT_LONGS_EQUAL(2, layout.blocks().size());

  EXPECT_LONGS_EQUAL(kPoseKey, layout.blocks()[0].key);
  EXPECT_LONGS_EQUAL(3, layout.blocks()[0].dimension);
  EXPECT_LONGS_EQUAL(0, layout.blocks()[0].columnBegin);

  EXPECT_LONGS_EQUAL(kPointKey, layout.blocks()[1].key);
  EXPECT_LONGS_EQUAL(2, layout.blocks()[1].dimension);
  EXPECT_LONGS_EQUAL(3, layout.blocks()[1].columnBegin);

  EXPECT_LONGS_EQUAL(0, layout.at(kPoseKey).columnBegin);
  EXPECT_LONGS_EQUAL(3, layout.at(kPointKey).columnBegin);

  const Vector flatDelta = (Vector(5) << 1.0, 2.0, 3.0, 4.0, 5.0).finished();
  const VectorValues delta = layout.toVectorValues(flatDelta);
  EXPECT(assert_equal(Vector3(1.0, 2.0, 3.0), delta.at(kPoseKey)));
  EXPECT(assert_equal(Vector2(4.0, 5.0), delta.at(kPointKey)));
}

TEST(SparseJacobianColumnLayout, RejectsUnknownKeyAndWrongDeltaSize) {
  const SparseJacobianColumnLayout layout(makeValues());

  CHECK_EXCEPTION(layout.at(999), std::out_of_range);
  CHECK_EXCEPTION(layout.toVectorValues(Vector::Zero(4)),
                  std::invalid_argument);
}

TEST(SparseJacobianColumnLayout, MatchesTheCompleteKeyDimensionSequence) {
  const SparseJacobianColumnLayout layout(makeValues());
  CHECK(layout.matches(makeValues()));

  Values changedValues;
  changedValues.insert(kPoseKey, Pose2());
  changedValues.insert(kPointKey, Point3(4.0, 5.0, 6.0));
  CHECK(!layout.matches(changedValues));
}

TEST(SparseJacobianPlan, BuildsScalarCsrInGlobalColumnOrder) {
  const Values values = makeValues();
  const SparseJacobianColumnLayout layout(values);
  const SparseJacobianPlan plan(makeGraph(), layout);

  EXPECT_LONGS_EQUAL(5, plan.rows());
  EXPECT_LONGS_EQUAL(5, plan.columns());
  EXPECT_LONGS_EQUAL(19, plan.nonzeros());

  const std::vector<int> expectedRowPointers = {0, 3, 6, 9, 14, 19};
  const std::vector<int> expectedColumnIndices = {
      0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
  CHECK(plan.rowPointers() == expectedRowPointers);
  CHECK(plan.columnIndices() == expectedColumnIndices);

  const SparseJacobianFactorWritePlan& reversed = plan.factor(1);
  EXPECT_LONGS_EQUAL(3, reversed.rowBegin);
  EXPECT_LONGS_EQUAL(2, reversed.rowCount);
  EXPECT_LONGS_EQUAL(5, reversed.nonzerosPerRow);
  EXPECT_LONGS_EQUAL(2, reversed.blocks.size());

  EXPECT_LONGS_EQUAL(kPointKey, reversed.blocks[0].key);
  EXPECT_LONGS_EQUAL(0, reversed.blocks[0].localBlockIndex);
  EXPECT_LONGS_EQUAL(2, reversed.blocks[0].width);
  EXPECT_LONGS_EQUAL(3, reversed.blocks[0].globalColumnBegin);
  EXPECT_LONGS_EQUAL(3, reversed.blocks[0].valueOffsetWithinRow);

  EXPECT_LONGS_EQUAL(kPoseKey, reversed.blocks[1].key);
  EXPECT_LONGS_EQUAL(1, reversed.blocks[1].localBlockIndex);
  EXPECT_LONGS_EQUAL(3, reversed.blocks[1].width);
  EXPECT_LONGS_EQUAL(0, reversed.blocks[1].globalColumnBegin);
  EXPECT_LONGS_EQUAL(0, reversed.blocks[1].valueOffsetWithinRow);

  CHECK_EXCEPTION(plan.factor(2), std::out_of_range);
}

TEST(SparseJacobianPlan, RejectsMissingAndRepeatedFactorKeys) {
  const SparseJacobianColumnLayout layout(makeValues());

  NonlinearFactorGraph missing;
  missing.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  missing.emplace_shared<ReversedPointPoseFactor>(999, kPoseKey);
  CHECK_EXCEPTION(SparseJacobianPlan(missing, layout), std::invalid_argument);

  NonlinearFactorGraph repeated;
  repeated.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  repeated.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPointKey, kPointKey}, 2));
  CHECK_EXCEPTION(SparseJacobianPlan(repeated, layout), std::invalid_argument);
}

TEST(SparseJacobianPlan, RejectsAnUncoveredValuesKey) {
  const SparseJacobianColumnLayout layout(makeValues());

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());

  bool exceptionNamesKey = false;
  try {
    const SparseJacobianPlan plan(graph, layout);
    (void)plan;
  } catch (const std::invalid_argument& error) {
    exceptionNamesKey =
        std::string(error.what()).find(DefaultKeyFormatter(kPointKey)) !=
        std::string::npos;
  }
  CHECK(exceptionNamesKey);
}

TEST(SparseJacobianPlan, ZeroRowFactorDoesNotProvideStructuralCoverage) {
  const SparseJacobianColumnLayout layout(makeValues());

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  graph.push_back(
      std::make_shared<StructuralFactor>(KeyVector{kPointKey}, 0));
  CHECK_EXCEPTION(SparseJacobianPlan(graph, layout), std::invalid_argument);
}

TEST(SparseJacobianPlan, RejectsFactorCountsOutsideSignedIntCapacity) {
  Values values;
  values.insert(kPointKey, Point2(4.0, 5.0));
  const SparseJacobianColumnLayout layout(values);

  NonlinearFactorGraph rowOverflow;
  rowOverflow.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPointKey},
      static_cast<size_t>(std::numeric_limits<int>::max()) + 1U));
  CHECK_EXCEPTION(SparseJacobianPlan(rowOverflow, layout),
                  std::invalid_argument);

  NonlinearFactorGraph nonzeroOverflow;
  nonzeroOverflow.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPointKey},
      static_cast<size_t>(std::numeric_limits<int>::max())));
  CHECK_EXCEPTION(SparseJacobianPlan(nonzeroOverflow, layout),
                  std::invalid_argument);
}

TEST(SparseJacobianPlan, PreservesNullSlotsAndLaterRowOffsets) {
  const Values values = makeValues();
  const SparseJacobianColumnLayout layout(values);

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  graph.push_back(std::shared_ptr<NonlinearFactor>());
  graph.emplace_shared<ReversedPointPoseFactor>(kPointKey, kPoseKey);

  const SparseJacobianPlan plan(graph, layout);
  EXPECT_LONGS_EQUAL(3, plan.factors().size());
  EXPECT_LONGS_EQUAL(3, plan.factor(1).rowBegin);
  EXPECT_LONGS_EQUAL(0, plan.factor(1).rowCount);
  EXPECT_LONGS_EQUAL(0, plan.factor(1).nonzerosPerRow);
  EXPECT_LONGS_EQUAL(0, plan.factor(1).blocks.size());
  EXPECT_LONGS_EQUAL(3, plan.factor(2).rowBegin);
  EXPECT_LONGS_EQUAL(2, plan.factor(2).rowCount);
  EXPECT_LONGS_EQUAL(5, plan.rows());
  EXPECT_LONGS_EQUAL(19, plan.nonzeros());
}

TEST(SparseJacobianPlan, MatchesOnlyIdenticalStructure) {
  const Values values = makeValues();
  const SparseJacobianColumnLayout layout(values);
  const NonlinearFactorGraph graph = makeGraph();
  const SparseJacobianPlan plan(graph, layout);

  const Values identicalValues = makeValues();
  const SparseJacobianColumnLayout identicalLayout(identicalValues);
  const NonlinearFactorGraph identicalGraph = makeGraph();
  const SparseJacobianPlan identicalPlan(identicalGraph, identicalLayout);
  CHECK(plan.matches(identicalGraph, identicalLayout));
  CHECK(plan.structuralFingerprint() ==
        identicalPlan.structuralFingerprint());

  Values changedValues;
  changedValues.insert(kPoseKey, Pose2());
  changedValues.insert(kPointKey, Point3(0.0, 0.0, 0.0));
  const SparseJacobianColumnLayout changedLayout(changedValues);
  CHECK(!plan.matches(graph, changedLayout));

  NonlinearFactorGraph reorderedGraph;
  reorderedGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  reorderedGraph.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPoseKey, kPointKey}, 2));
  const SparseJacobianPlan reorderedPlan(reorderedGraph, layout);
  CHECK(!plan.matches(reorderedGraph, layout));
  CHECK(plan.structuralFingerprint() !=
        reorderedPlan.structuralFingerprint());

  NonlinearFactorGraph unsendableGraph;
  unsendableGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  unsendableGraph.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPointKey, kPoseKey}, 2, false));
  const SparseJacobianPlan unsendablePlan(unsendableGraph, layout);
  CHECK(!plan.matches(unsendableGraph, layout));
  CHECK(plan.structuralFingerprint() !=
        unsendablePlan.structuralFingerprint());

  NonlinearFactorGraph changedRowsGraph;
  changedRowsGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  changedRowsGraph.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPointKey, kPoseKey}, 3));
  const SparseJacobianPlan changedRowsPlan(changedRowsGraph, layout);
  CHECK(!plan.matches(changedRowsGraph, layout));
  CHECK(plan.structuralFingerprint() !=
        changedRowsPlan.structuralFingerprint());
}

TEST(SparseJacobianPlan, FingerprintAndMatchesIncludeNullMarkers) {
  const SparseJacobianColumnLayout layout(makeValues());

  NonlinearFactorGraph nullGraph;
  nullGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  nullGraph.push_back(std::shared_ptr<NonlinearFactor>());
  nullGraph.emplace_shared<ReversedPointPoseFactor>(kPointKey, kPoseKey);
  const SparseJacobianPlan nullPlan(nullGraph, layout);

  NonlinearFactorGraph zeroRowGraph;
  zeroRowGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  zeroRowGraph.push_back(
      std::make_shared<StructuralFactor>(KeyVector{}, 0));
  zeroRowGraph.emplace_shared<ReversedPointPoseFactor>(kPointKey, kPoseKey);
  const SparseJacobianPlan zeroRowPlan(zeroRowGraph, layout);

  CHECK(!nullPlan.matches(zeroRowGraph, layout));
  CHECK(nullPlan.structuralFingerprint() !=
        zeroRowPlan.structuralFingerprint());
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
