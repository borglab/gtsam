/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cyclic.h
 * @brief  Cyclic group, i.e., the integers modulo N
 * @author Frank Dellaert
 **/

#include <gtsam/base/concepts.h>
#include <cstddef>

namespace gtsam {

template<size_t N>
class Cyclic {
  size_t i_; ///< we just use an unsigned int
public:
  /// Constructor
  Cyclic(size_t i) :
      i_(i) {
  }
  /// Cast to size_t
  operator size_t() const {
    return i_;
  }
};

namespace traits {
template<size_t N> struct structure_category<Cyclic<N> > {
  typedef group_tag type;
};
template<size_t N> struct identity<Cyclic<N> > {
  static const Cyclic<N> value;
  typedef Cyclic<N> value_type;
};
template<size_t N> struct group_flavor<Cyclic<N> > {
  typedef additive_group_tag type;
};
} // \namespace traits

} // \namespace gtsam

/**
 * @file   Cyclic.cpp
 * @brief  Cyclic group implementation
 * @author Frank Dellaert
 **/

namespace gtsam {

namespace traits {
template<size_t N>
const Cyclic<N> identity<Cyclic<N> >::value = Cyclic<N>(0);
} // \namespace traits

} // \namespace gtsam

/**
 * @file   testCyclic.cpp
 * @brief  Unit tests for cyclic group
 * @author Frank Dellaert
 **/

//#include <gtsam/geometry/Cyclic.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Cyclic<6> G; // Let's use the cyclic group of order 6

//******************************************************************************
TEST(Cyclic, Concept) {
  EXPECT_LONGS_EQUAL(0,traits::identity<G>::value);
  BOOST_CONCEPT_ASSERT((Group<G>));
}

//******************************************************************************
TEST(Cyclic, Constructor) {
  G g(0);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

