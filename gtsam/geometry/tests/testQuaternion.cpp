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

namespace traits {
/// Define Eigen::Quaternion to be a model of the Group concept
template<typename S, int O>
struct structure_category<Eigen::Quaternion<S, O> > {
  typedef group_tag type;
};
} // \namespace gtsam::traits

namespace group {

template<typename S, int O>
Eigen::Quaternion<S, O> compose(const Eigen::Quaternion<S, O> &g,
    const Eigen::Quaternion<S, O> & h) {
  return g * h;
}

template<typename S, int O>
Eigen::Quaternion<S, O> between(const Eigen::Quaternion<S, O> &g,
    const Eigen::Quaternion<S, O> & h) {
  return g.inverse() * h;
}

template<typename S, int O>
Eigen::Quaternion<S, O> inverse(const Eigen::Quaternion<S, O> &g) {
  return g.inverse();
}

namespace traits {

/// Declare the trait that specifies a quaternion's identity element
template<typename S, int O>
struct identity<Eigen::Quaternion<S, O> > {
  static const Eigen::Quaternion<S, O> value;
  typedef Eigen::Quaternion<S, O> value_type;
};

/// Out of line definition of identity
template<typename S, int O>
const Eigen::Quaternion<S, O> identity<Eigen::Quaternion<S, O> >::value =
    Eigen::Quaternion<S, O>::Identity();

/// Define the trait that asserts quaternions are a multiplicative group
template<typename S, int O>
struct flavor<Eigen::Quaternion<S, O> > {
  typedef multiplicative_tag type;
};

} // \namespace gtsam::group::traits
} // \namespace gtsam::group
} // \namespace gtsam

/**
 *  GSAM typedef to an Eigen::Quaternion<double>, we disable alignment because
 *  geometry objects are stored in boost pool allocators, in Values containers,
 *  and and these pool allocators do not support alignment.
 */
typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

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
TEST(Quaternion , Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<Quaternion >));
}

//******************************************************************************
TEST(Quaternion , Constructor) {
  Q g(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
}

//******************************************************************************
TEST(Quaternion , Compose) {
}

//******************************************************************************
TEST(Quaternion , Between) {
}

//******************************************************************************
TEST(Quaternion , Ivnverse) {
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

