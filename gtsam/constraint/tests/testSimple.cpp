#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <iostream>
#include <functional>

// Define a functor class
class MyFunctor {
public:
    // Overload the function call operator
    void operator()(int x) const {
        std::cout << "MyFunctor called with " << x << std::endl;
    }
};


TEST(InequalityConstraint, DoubleExpressionInequality) {
    // Create an instance of the functor
    MyFunctor functor;

    // Cast the functor to std::function
    std::function<void(int)> func = functor;

    // Call the std::function object
    func(42);
}


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
