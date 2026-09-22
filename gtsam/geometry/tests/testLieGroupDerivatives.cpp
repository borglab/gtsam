/**
 * @file testLieGroupDerivatives.cpp
 * @brief Verify that Lie-group traits expose only implemented derivatives.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>

#include <type_traits>
#include <utility>

using namespace gtsam;

/* ************************************************************************* */
namespace lie_group_derivative_support {

template <class T>
using LocalJacobian = decltype(traits<T>::Local(
    std::declval<const T&>(), std::declval<const T&>(),
    std::declval<typename traits<T>::ChartJacobian>(),
    std::declval<typename traits<T>::ChartJacobian>()));

template <class T>
using RetractJacobian = decltype(traits<T>::Retract(
    std::declval<const T&>(),
    std::declval<const typename traits<T>::TangentVector&>(),
    std::declval<typename traits<T>::ChartJacobian>(),
    std::declval<typename traits<T>::ChartJacobian>()));

template <class T>
using ExpmapJacobian = decltype(traits<T>::Expmap(
    std::declval<const typename traits<T>::TangentVector&>(),
    std::declval<typename traits<T>::ChartJacobian>()));

template <class T>
using LogmapJacobian = decltype(traits<T>::Logmap(
    std::declval<const T&>(),
    std::declval<typename traits<T>::ChartJacobian>()));

template <class T, template <class> class Operation, class = void>
struct IsDetected : std::false_type {};

template <class T, template <class> class Operation>
struct IsDetected<T, Operation, std::void_t<Operation<T>>> : std::true_type {};

template <class T>
constexpr bool hasAllJacobians =
    IsDetected<T, LocalJacobian>::value &&
    IsDetected<T, RetractJacobian>::value &&
    IsDetected<T, ExpmapJacobian>::value &&
    IsDetected<T, LogmapJacobian>::value;

template <class T>
constexpr bool hasNoJacobians =
    !IsDetected<T, LocalJacobian>::value &&
    !IsDetected<T, RetractJacobian>::value &&
    !IsDetected<T, ExpmapJacobian>::value &&
    !IsDetected<T, LogmapJacobian>::value;

// Verifies that automatic Lie-group traits reflect derivative availability.
TEST(LieGroupDerivatives, TraitIntrospection) {
  CHECK(hasAllJacobians<SO3>);
  CHECK(hasNoJacobians<SO4>);
  CHECK(hasNoJacobians<Similarity2>);
  CHECK(hasNoJacobians<Similarity3>);
  CHECK(hasNoJacobians<SL4>);

  CHECK(internal::HasLocalJacobians<SO3>::value);
  CHECK(!internal::HasLocalJacobians<SO4>::value);
  CHECK(!internal::HasLocalJacobians<Similarity2>::value);
  CHECK(!internal::HasLocalJacobians<Similarity3>::value);
  CHECK(!internal::HasLocalJacobians<SL4>::value);
}

}  // namespace lie_group_derivative_support
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
/* ************************************************************************* */
