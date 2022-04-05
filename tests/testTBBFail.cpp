#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>              // Bug occurs
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>         // Bug occurs
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>  // Bug occurs
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <ostream>

#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/InitializePose3.h"

#define BOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED

#include <signal.h>  // ::signal, ::raise

#include <boost/stacktrace.hpp>

void my_signal_handler(int signum) {
  ::signal(signum, SIG_DFL);
  std::cout << boost::stacktrace::stacktrace();
	std::flush(std::cout);

  ::raise(SIGTERM);
}

TEST_UNSAFE(TBB, MemoryCorruption) {
  // Read g2o input file
  gtsam::NonlinearFactorGraph::shared_ptr graph_;
  gtsam::Values::shared_ptr initial_;

  boost::tie(graph_, initial_) =
      gtsam::load3D(gtsam::findExampleDataFile("sphere2500"));

  gtsam::NonlinearFactorGraph graph(graph_->begin(), graph_->begin() + 500);
  //   // Add prior
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      250, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-1));
  auto initial = gtsam::InitializePose3::initialize(graph);

  // Run the optimizer a bunch
  for (size_t i = 0; i < 1000; i++) {
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();
    // result.print();
    std::cout << "Iteration " << i << "\n";
  }
}

/* ************************************************************************* */
int main() {
  ::signal(SIGSEGV, &my_signal_handler);
  ::signal(SIGBUS, &my_signal_handler);
  ::signal(SIGABRT, &my_signal_handler);

  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
