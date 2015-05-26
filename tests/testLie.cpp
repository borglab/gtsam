/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testLie.cpp
 * @date May, 2015
 * @author Frank Dellaert
 * @brief unit tests for Lie group type machinery
 */

#include <gtsam/base/Lie.h>
#include <utility>

namespace gtsam {

/// Template to construct the product Lie group of two other Lie groups, G and H
/// Assumes manifold structure from G and H, and binary constructor
template<typename G, typename H>
class ProductLieGroup: public std::pair<G, H>, public LieGroup<
    ProductLieGroup<G, H>, traits<G>::dimension + traits<H>::dimension> {
  BOOST_CONCEPT_ASSERT((IsLieGroup<G>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<H>));
  typedef std::pair<G, H> Base;

protected:
  enum {dimension1 = traits<G>::dimension};
  enum {dimension2 = traits<H>::dimension};

public:
  /// Default constructor yields identity
  ProductLieGroup():Base(traits<G>::Identity(),traits<H>::Identity()) {}

  // Construct from two subgroup elements
  ProductLieGroup(const G& g, const H& h):Base(g,h) {}

  // Construct from base
  ProductLieGroup(const Base& base):Base(base) {}

  /// @name Group
  /// @{
  typedef multiplicative_group_tag group_flavor;
  static ProductLieGroup identity() {return ProductLieGroup();}

  ProductLieGroup operator*(const ProductLieGroup& other) const {
    return ProductLieGroup(traits<G>::Compose(this->first,other.first),
        traits<H>::Compose(this->second,other.second));
  }
  ProductLieGroup inverse() const {
    return ProductLieGroup(this->first.inverse(), this->second.inverse());
  }
  /// @}

  /// @name Manifold (but with derivatives)
  /// @{
  enum {dimension = dimension1 + dimension2};
  inline static size_t Dim() {return dimension;}
  inline size_t dim() const {return dimension;}

  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  /// @}

  /// @name Lie Group
  /// @{
  Eigen::Matrix<double,dimension,dimension> AdjointMap() const {
    Eigen::Matrix<double,dimension,dimension> A;
    A.setIdentity();
    throw std::runtime_error("ProductLieGroup::derivatives not implemented yet");
//    A.template topLeftCorner<dimension1, dimension1>() = this->first.AdjointMap();
//    A.template bottomRightCorner<dimension2, dimension2>() = this->second.AdjointMap();
    return A;
  }
  static ProductLieGroup Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {
    if (Hv) throw std::runtime_error("ProductLieGroup::derivatives not implemented yet");
    G g = traits<G>::Expmap(v.template head<dimension1>());
    H h = traits<H>::Expmap(v.template tail<dimension2>());
    return ProductLieGroup(g,h);
  }
  static TangentVector Logmap(const ProductLieGroup& p, ChartJacobian Hp = boost::none) {
    if (Hp) throw std::runtime_error("ProductLieGroup::derivatives not implemented yet");
    typename traits<G>::TangentVector v1 = traits<G>::Logmap(p.first);
    typename traits<H>::TangentVector v2 = traits<H>::Logmap(p.second);
    TangentVector v;
    v << v1, v2;
    return v;
  }
  struct ChartAtOrigin {
    static TangentVector Local(const ProductLieGroup& m, ChartJacobian Hm = boost::none) {
      return Logmap(m, Hm);
    }
    static ProductLieGroup Retract(const TangentVector& v, ChartJacobian Hv = boost::none) {
      return Expmap(v, Hv);
    }
  };
  using LieGroup<ProductLieGroup,dimension>::inverse; // with derivative
  /// @}
};

// Define any direct product group to be a model of the multiplicative Group concept
template<typename G, typename H>
struct traits<ProductLieGroup<G, H> > : internal::LieGroupTraits<
    ProductLieGroup<G, H> > {
};
}
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/testLie.h>

#undef CHECK
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//******************************************************************************
typedef ProductLieGroup<Point2, Point2> MyPoint2Pair;

// Define any direct product group to be a model of the multiplicative Group concept
namespace gtsam {
template<> struct traits<MyPoint2Pair> : internal::LieGroupTraits<MyPoint2Pair> {
  static void Print(const MyPoint2Pair& m, const string& s = "") {
    cout << s << "(" << m.first << "," << m.second << ")" << endl;
  }
  static bool Equals(const MyPoint2Pair& m1, const MyPoint2Pair& m2,
      double tol = 1e-8) {
    return m1 == m2;
  }
};
}

TEST(Lie, ProductLieGroup) {
  BOOST_CONCEPT_ASSERT((IsGroup<MyPoint2Pair>));
  BOOST_CONCEPT_ASSERT((IsManifold<MyPoint2Pair>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<MyPoint2Pair>));
  MyPoint2Pair pair1;
  Vector4 d;
  d << 1, 2, 3, 4;
  MyPoint2Pair expected(Point2(1, 2), Point2(3, 4));
  MyPoint2Pair pair2 = pair1.retract(d);
  EXPECT(assert_equal(expected, pair2, 1e-9));
  EXPECT(assert_equal(d, pair1.localCoordinates(pair2), 1e-9));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

