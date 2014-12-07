/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Quaternion.h
 * @brief  Unit tests for unit quaternions
 * @author Frank Dellaert
 **/

#include <gtsam/base/concepts.h>

namespace gtsam {

/// Typedef to an Eigen Quaternion<double>, we disable alignment because
/// geometry objects are stored in boost pool allocators, in Values
/// containers, and and these pool allocators do not support alignment.
typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

namespace traits {
/// Define Quaternion to be a model of the Group concept
template<>
struct structure_category<Quaternion> {
  typedef group_tag type;
};
} // \namespace gtsam::traits

namespace group {

Quaternion compose(const Quaternion&g, const Quaternion& h) {
  return g * h;
}

Quaternion between(const Quaternion&g, const Quaternion& h) {
  return g.inverse() * h;
}

Quaternion inverse(const Quaternion&g) {
  return g.inverse();
}

namespace traits {

/// Define the trait that specifies Quaternion's identity element
template<>
struct identity<Quaternion> {
  static const Quaternion value;
  typedef Quaternion value_type;
};

const Quaternion identity<Quaternion>::value = Quaternion::Identity();

/// Define the trait that asserts Quaternion is an additive group
template<>
struct flavor<Quaternion> {
  typedef multiplicative_tag type;
};

} // \namespace gtsam::group::traits
} // \namespace gtsam::group
} // \namespace gtsam

/**
 * @file   testCyclic.cpp
 * @brief  Unit tests for cyclic group
 * @author Frank Dellaert
 **/

//#include <gtsam/geometry/Quaternion.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Quaternion Q; // Typedef

//******************************************************************************
TEST(Quaternion, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<Quaternion>));
}

//******************************************************************************
TEST(Quaternion, Constructor) {
  Q g(Eigen::AngleAxisd(1, Vector3(0,0,1)));
}

//******************************************************************************
TEST(Quaternion, Compose) {
}

//******************************************************************************
TEST(Quaternion, Between) {
}

//******************************************************************************
TEST(Quaternion, Ivnverse) {
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

