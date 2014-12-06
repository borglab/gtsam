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
  /// Addition modulo N
  Cyclic operator+(const Cyclic& h) const {
    return (i_+h.i_) % N;
  }
};

namespace traits {
/// Define Cyclic to be a model of the Group concept
template<size_t N> struct structure_category<Cyclic<N> > {
  typedef group_tag type;
};
} // \namespace traits

namespace group {

template<size_t N>
Cyclic<N> compose(const Cyclic<N>&g, const Cyclic<N>& h) {
  return g + h;
}

template<size_t N>
Cyclic<N> between(const Cyclic<N>&g, const Cyclic<N>& h) {
  return h - g;
}

template<size_t N>
Cyclic<N> inverse(const Cyclic<N>&g) {
  return -g;
}

namespace traits {
/// Define the trait that specifies Cyclic's identity element
template<size_t N> struct identity<Cyclic<N> > {
  static const Cyclic<N> value;
  typedef Cyclic<N> value_type;
};
/// Define the trait that asserts Cyclic is an additive group
template<size_t N> struct flavor<Cyclic<N> > {
  typedef additive_tag type;
};
} // \namespace traits
} // \namespace group

} // \namespace gtsam

/**
 * @file   Cyclic.cpp
 * @brief  Cyclic group implementation
 * @author Frank Dellaert
 **/

namespace gtsam {

namespace group {
namespace traits {
template<size_t N>
const Cyclic<N> identity<Cyclic<N> >::value = Cyclic<N>(0);
} // \namespace traits
} // \namespace group

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
  BOOST_CONCEPT_ASSERT((Group<G>));
  EXPECT_LONGS_EQUAL(0, group::traits::identity<G>::value);
  G g(2), h(3);
  // EXPECT(Group<G>().check_invariants(g,h))
}

//******************************************************************************
TEST(Cyclic, Constructor) {
  G g(0);
}

//******************************************************************************
TEST(Cyclic, Compose) {
  G e(0), g(2), h(3);
  EXPECT_LONGS_EQUAL(5, group::compose(g,h));
  EXPECT_LONGS_EQUAL(0, group::compose(h,h));
  EXPECT_LONGS_EQUAL(3, group::compose(h,e));
}

//******************************************************************************
TEST(Cyclic, Between) {
  G g(2), h(3);
  EXPECT_LONGS_EQUAL(1, group::between(g,h));
  EXPECT_LONGS_EQUAL(0, group::between(g,g));
  EXPECT_LONGS_EQUAL(0, group::between(h,h));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

