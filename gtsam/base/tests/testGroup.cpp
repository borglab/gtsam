/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testGroup.cpp
 * @brief  Unit tests for groups
 * @author Frank Dellaert
 **/

#include <gtsam/base/Group.h>
#include <gtsam/base/Testable.h>
#include <Eigen/Core>
#include <iostream>

namespace gtsam {

/// Symmetric group
template<int N>
class Symmetric: private Eigen::PermutationMatrix<N> {
  Symmetric(const Eigen::PermutationMatrix<N>& P) :
      Eigen::PermutationMatrix<N>(P) {
  }
public:
  static Symmetric Identity() { return Symmetric(); }
  Symmetric() {
    Eigen::PermutationMatrix<N>::setIdentity();
  }
  static Symmetric Transposition(int i, int j) {
    Symmetric g;
    return g.applyTranspositionOnTheRight(i, j);
  }
  Symmetric operator*(const Symmetric& other) const {
    return Eigen::PermutationMatrix<N>::operator*(other);
  }
  bool operator==(const Symmetric& other) const {
    for (size_t i = 0; i < N; i++)
      if (this->indices()[i] != other.indices()[i])
        return false;
    return true;
  }
  Symmetric inverse() const {
    return Eigen::PermutationMatrix<N>(Eigen::PermutationMatrix<N>::inverse());
  }
  friend std::ostream &operator<<(std::ostream &os, const Symmetric& m) {
    for (size_t i = 0; i < N; i++)
      os << m.indices()[i] << " ";
    return os;
  }
  void print(const std::string& s = "") const {
    std::cout << s << *this << std::endl;
  }
  bool equals(const Symmetric<N>& other, double tol = 0) const {
    return this->indices() == other.indices();
  }
};

/// Define permutation group traits to be a model of the Multiplicative Group concept
template<int N>
struct traits<Symmetric<N> > : internal::MultiplicativeGroupTraits<Symmetric<N> >,
    Testable<Symmetric<N> > {
};

} // namespace gtsam

#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//******************************************************************************
typedef Symmetric<2> S2;
TEST(Group, S2) {
  S2 e, s1 = S2::Transposition(0, 1);
  BOOST_CONCEPT_ASSERT((IsGroup<S2>));
  EXPECT(check_group_invariants(e, s1));
}

//******************************************************************************
typedef Symmetric<3> S3;
TEST(Group, S3) {
  S3 e, s1 = S3::Transposition(0, 1), s2 = S3::Transposition(1, 2);
  BOOST_CONCEPT_ASSERT((IsGroup<S3>));
  EXPECT(check_group_invariants(e, s1));
  EXPECT(assert_equal(s1, s1 * e));
  EXPECT(assert_equal(s1, e * s1));
  EXPECT(assert_equal(e, s1 * s1));
  S3 g = s1 * s2; // 1 2 0
  EXPECT(assert_equal(s1, g * s2));
  EXPECT(assert_equal(e, compose_pow(g, 0)));
  EXPECT(assert_equal(g, compose_pow(g, 1)));
  EXPECT(assert_equal(e, compose_pow(g, 3))); // g is generator of Z3 subgroup
}

//******************************************************************************
// The direct product of S2=Z2 and S3 is the symmetry group of a hexagon,
// i.e., the dihedral group of order 12 (denoted Dih6 because 6-sided polygon)
namespace gtsam {
typedef DirectProduct<S2, S3> Dih6;

std::ostream &operator<<(std::ostream &os, const Dih6& m) {
  os << "( " << m.first << ", " << m.second << ")";
  return os;
}

// Provide traits with Testable

template<>
struct traits<Dih6> : internal::MultiplicativeGroupTraits<Dih6> {
  static void Print(const Dih6& m, const string& s = "") {
    cout << s << m << endl;
  }
  static bool Equals(const Dih6& m1, const Dih6& m2, double tol = 1e-8) {
    return m1 == m2;
  }
};
} // namespace gtsam

TEST(Group, Dih6) {
  Dih6 e, g(S2::Transposition(0, 1),
      S3::Transposition(0, 1) * S3::Transposition(1, 2));
  BOOST_CONCEPT_ASSERT((IsGroup<Dih6>));
  EXPECT(check_group_invariants(e, g));
  EXPECT(assert_equal(e, compose_pow(g, 0)));
  EXPECT(assert_equal(g, compose_pow(g, 1)));
  EXPECT(assert_equal(e, compose_pow(g, 6))); // g is generator of Z6 subgroup
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

