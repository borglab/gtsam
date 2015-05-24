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

#include <gtsam/geometry/Cyclic.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Cyclic<3> G; // Let's use the cyclic group of order 3

//******************************************************************************
TEST(Cyclic, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<G>));
  EXPECT_LONGS_EQUAL(0, traits<G>::Identity());
}

//******************************************************************************
TEST(Cyclic, Constructor) {
  G g(0);
}

//******************************************************************************
TEST(Cyclic, Compose) {
  EXPECT_LONGS_EQUAL(0, traits<G>::Compose(G(0),G(0)));
  EXPECT_LONGS_EQUAL(1, traits<G>::Compose(G(0),G(1)));
  EXPECT_LONGS_EQUAL(2, traits<G>::Compose(G(0),G(2)));

  EXPECT_LONGS_EQUAL(2, traits<G>::Compose(G(2),G(0)));
  EXPECT_LONGS_EQUAL(0, traits<G>::Compose(G(2),G(1)));
  EXPECT_LONGS_EQUAL(1, traits<G>::Compose(G(2),G(2)));
}

//******************************************************************************
TEST(Cyclic, Between) {
  EXPECT_LONGS_EQUAL(0, traits<G>::Between(G(0),G(0)));
  EXPECT_LONGS_EQUAL(1, traits<G>::Between(G(0),G(1)));
  EXPECT_LONGS_EQUAL(2, traits<G>::Between(G(0),G(2)));

  EXPECT_LONGS_EQUAL(1, traits<G>::Between(G(2),G(0)));
  EXPECT_LONGS_EQUAL(2, traits<G>::Between(G(2),G(1)));
  EXPECT_LONGS_EQUAL(0, traits<G>::Between(G(2),G(2)));
}

//******************************************************************************
TEST(Cyclic, Inverse) {
  EXPECT_LONGS_EQUAL(0, traits<G>::Inverse(G(0)));
  EXPECT_LONGS_EQUAL(2, traits<G>::Inverse(G(1)));
  EXPECT_LONGS_EQUAL(1, traits<G>::Inverse(G(2)));
}

//******************************************************************************
TEST(Cyclic, Negation) {
  EXPECT_LONGS_EQUAL(0, -G(0));
  EXPECT_LONGS_EQUAL(2, -G(1));
  EXPECT_LONGS_EQUAL(1, -G(2));
}

//******************************************************************************
TEST(Cyclic, Negation2) {
  typedef Cyclic<2> Z2;
  EXPECT_LONGS_EQUAL(0, -Z2(0));
  EXPECT_LONGS_EQUAL(1, -Z2(1));
}

//******************************************************************************
TEST(Cyclic , Invariants) {
  G g(2), h(1);
 check_group_invariants(g,h);
}

//******************************************************************************
namespace gtsam {

template<typename G, typename H>
class DirectSum: public std::pair<G, H> {
  BOOST_CONCEPT_ASSERT((IsGroup<G>));  // TODO(frank): check additive
  BOOST_CONCEPT_ASSERT((IsGroup<H>));  // TODO(frank): check additive
  BOOST_CONCEPT_ASSERT((IsTestable<G>));
  BOOST_CONCEPT_ASSERT((IsTestable<H>));

  const G& g() const { return this->first; }
  const H& h() const { return this->second;}

public:
  // Construct from two subgroup elements
  DirectSum(const G& g, const H& h):std::pair<G,H>(g,h) {}

  /// Default constructor yields identity
  DirectSum():std::pair<G,H>(G::Identity(),H::Identity()) {
  }

  /// Identity element
  static DirectSum Identity() {
    return DirectSum();
  }
  /// Addition
  DirectSum operator+(const DirectSum& other) const {
    return DirectSum(g()+other.g(), h()+other.h());
  }
  /// Subtraction
  DirectSum operator-(const DirectSum& other) const {
    return DirectSum(g()-other.g(), h()-other.h());
  }
  /// Inverse
  DirectSum operator-() const {
    return DirectSum(- g(), - h());
  }
  /// print with optional string
  void print(const std::string& s = "") const {
    std::cout << s << "(\n";
    traits<G>::Print(g());
    std::cout << ",\n";
    traits<H>::Print(h());
    std::cout << ")" << std::endl;
  }
  /// equals with an tolerance, prints out message if unequal
  bool equals(const DirectSum& other, double tol = 1e-9) const {
    return *this == other; // uses std::pair operator
  }
};

/// Define direct sums to be a model of the Additive Group concept
template<typename G, typename H>
struct traits<DirectSum<G, H> > : internal::AdditiveGroupTraits<DirectSum<G, H> >, //
    Testable<DirectSum<G, H> > {
};

}  // namespace gtsam

TEST(Cyclic , DirectSum) {
  // The Direct sum of Cyclic<2> and Cyclic<2> is *not* Cyclic<4>, but the
  // smallest non-cyclic group called the Klein four-group:
  typedef DirectSum<Cyclic<2>, Cyclic<2> > K;
  BOOST_CONCEPT_ASSERT((IsGroup<K>));
  BOOST_CONCEPT_ASSERT((IsTestable<K>));

  K g(0, 1), h(1, 1);
  EXPECT(assert_equal(K(0, 1), - g));
  EXPECT(assert_equal(K(), g + g));
  EXPECT(assert_equal(K(1, 0), g + h));
  EXPECT(assert_equal(K(1, 0), g - h));
  check_group_invariants(g, h);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

