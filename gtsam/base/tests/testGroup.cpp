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
#include <Eigen/Core>
#include <iostream>

namespace gtsam {

template<int N>
class Permutation: public Eigen::PermutationMatrix<N> {
public:
  Permutation() {
    Eigen::PermutationMatrix<N>::setIdentity();
  }
  Permutation(const Eigen::PermutationMatrix<N>& P) :
      Eigen::PermutationMatrix<N>(P) {
  }
  Permutation inverse() const {
    return Eigen::PermutationMatrix<N>(Eigen::PermutationMatrix<N>::inverse());
  }
};

/// Define permutation group traits to be a model of the Multiplicative Group concept
template<int N>
struct traits<Permutation<N> > : internal::MultiplicativeGroupTraits<
    Permutation<N> > {
  static void Print(const Permutation<N>& m, const std::string& str = "") {
    std::cout << m << std::endl;
  }
  static bool Equals(const Permutation<N>& m1, const Permutation<N>& m2,
      double tol = 1e-8) {
    return m1.indices() == m2.indices();
  }
};

} // namespace gtsam

#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Permutation<3> G; // Let's use the permutation group of order 3

//******************************************************************************
TEST(Group, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<G>));
}

//******************************************************************************
TEST(Group , Invariants) {
  G g, h;
  EXPECT(check_group_invariants(g, h));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

