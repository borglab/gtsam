/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testCyclic.cpp
 * @brief  Unit tests for cyclic group
 * @author Frank Dellaert
 **/

#include <gtsam/base/concepts.h>
#include <cstddef>

namespace gtsam {

template<size_t N>
class Cyclic {
  size_t i_;
public:
  static const Cyclic Identity = Cyclic(0);
  Cyclic(size_t i) :
      i_(i) {
  }
};

namespace traits {
template<size_t N> struct identity<Cyclic<N> > {
  static const Cyclic<N> value = Cyclic<N>::Identity;
  typedef Cyclic<N> value_type;
};
template<size_t N> struct group_flavor<Cyclic<N> > {
  typedef additive_group_tag type;
};
} // \namespace traits

} // \namespace gtsam

//#include <gtsam/geometry/Cyclic.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

BOOST_CONCEPT_ASSERT((GroupConcept<Cyclic<6> >));

/* ************************************************************************* */
TEST(Cyclic, Constructor) {
Cyclic<6> g(0);
//  EXPECT(assert_equal(p1, p2));
//  EXPECT_LONGS_EQUAL(2,offset2.size());
}

/* ************************************************************************* */
int main() {
TestResult tr;
return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

