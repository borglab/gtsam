/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testStagMPCController.cpp
 * @brief   Unit tests for the StagMPCController base class and parameters
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/planning/controller/StagMPCController.h>

#include <memory>
#include <type_traits>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace pass_through {

/// Parameters of the test controller: the base ones plus one of its own.
struct PassThroughParams : public StagMPCControllerParams {
  double controlValue = 0.0;  ///< Value written to every control variable.
};

/**
 * A minimal controller for testing the base class: it returns the initial
 * values with every control set to params().controlValue.
 */
class PassThroughController : public StagMPCController {
 public:
  explicit PassThroughController(
      const PassThroughParams& params = PassThroughParams())
      : params_(params) {}

  /// The controller's own parameters.
  const PassThroughParams& params() const { return params_; }

  /// The parameters as the base class sees them, through _params().
  const StagMPCControllerParams& baseParams() const { return _params(); }

  /// Returns the initial values with every control set to controlValue.
  Values optimize(const NonlinearFactorGraph& /*futureGraph*/,
                  const Values& initial,
                  const KeyVector& controlKeys) override {
    Values result = initial;
    for (Key key : controlKeys) result.update(key, params_.controlValue);
    return result;
  }

 protected:
  const StagMPCControllerParams& _params() const override { return params_; }

 private:
  PassThroughParams params_;
};

// Checks that the base parameters default to verbose = false.
TEST(StagMPCController, DefaultParams) {
  const StagMPCControllerParams params;
  EXPECT(!params.verbose);
}

// Checks that equals compares the parameter values.
TEST(StagMPCController, ParamsEquals) {
  StagMPCControllerParams params, other;
  EXPECT(params.equals(other));
  other.verbose = true;
  EXPECT(!params.equals(other));
}

// Checks that a derived controller keeps its derived parameters and that
// _params() returns the same object as the base parameter type.
TEST(StagMPCController, DerivedParams) {
  PassThroughParams params;
  params.verbose = true;
  params.controlValue = 2.0;
  const PassThroughController controller(params);
  EXPECT(controller.params().verbose);
  EXPECT_DOUBLES_EQUAL(2.0, controller.params().controlValue, 0.0);
  EXPECT(&controller.baseParams() == &controller.params());
}

// Checks that optimize dispatches through a base-class reference and gets the
// graph's control keys.
TEST(StagMPCController, OptimizeThroughBase) {
  PassThroughParams params;
  params.controlValue = 1.5;
  PassThroughController controller(params);
  StagMPCController& base = controller;

  Values initial;
  initial.insert(X(0), 5.0);
  initial.insert(U(0), 0.0);
  initial.insert(U(1), 0.0);
  const Values result =
      base.optimize(NonlinearFactorGraph(), initial, KeyVector{U(0), U(1)});

  EXPECT_DOUBLES_EQUAL(5.0, result.at<double>(X(0)), 0.0);
  EXPECT_DOUBLES_EQUAL(1.5, result.at<double>(U(0)), 0.0);
  EXPECT_DOUBLES_EQUAL(1.5, result.at<double>(U(1)), 0.0);
}

// Checks that shared_ptr is std::shared_ptr to the base class and can hold a
// derived controller.
TEST(StagMPCController, SharedPointerType) {
  static_assert(std::is_same_v<StagMPCController::shared_ptr,
                               std::shared_ptr<StagMPCController>>);
  const StagMPCController::shared_ptr controller =
      std::make_shared<PassThroughController>();
  EXPECT(controller != nullptr);
}

// Checks that optimize dispatches through a shared_ptr, as when an MPC loop
// holds the controller, and that copies of the pointer share one controller.
TEST(StagMPCController, OptimizeThroughSharedPointer) {
  PassThroughParams params;
  params.controlValue = -0.5;
  const StagMPCController::shared_ptr controller =
      std::make_shared<PassThroughController>(params);
  const StagMPCController::shared_ptr holder = controller;
  EXPECT_LONGS_EQUAL(2, controller.use_count());

  Values initial;
  initial.insert(U(0), 0.0);
  const Values result =
      holder->optimize(NonlinearFactorGraph(), initial, KeyVector{U(0)});
  EXPECT_DOUBLES_EQUAL(-0.5, result.at<double>(U(0)), 0.0);
}

}  // namespace pass_through
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
