/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationLinear.cpp
 * @brief
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

/* ************************************************************************* */
// Noise model components
/* ************************************************************************* */

/* ************************************************************************* */
// Export Noisemodels
// See http://www.boost.org/doc/libs/1_32_0/libs/serialization/doc/special.html
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic")

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base, "gtsam_noiseModel_mEstimator_Base")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::GemanMcClure, "gtsam_noiseModel_mEstimator_GemanMcClure")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::TruncatedLeastSquares, "gtsam_noiseModel_mEstimator_TruncatedLeastSquares")

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel")
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal")

/* ************************************************************************* */
// example noise models
static noiseModel::Diagonal::shared_ptr diag3 = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.2, 0.3));
static noiseModel::Gaussian::shared_ptr gaussian3 = noiseModel::Gaussian::SqrtInformation(2.0 * I_3x3);
static noiseModel::Isotropic::shared_ptr iso3 = noiseModel::Isotropic::Sigma(3, 0.2);
static noiseModel::Constrained::shared_ptr constrained3 = noiseModel::Constrained::MixedSigmas(Vector3(0.0, 0.0, 0.1));
static noiseModel::Unit::shared_ptr unit3 = noiseModel::Unit::Create(3);

/* ************************************************************************* */
TEST (Serialization, noiseModels) {
  // tests using pointers to the derived class
  EXPECT(equalsDereferenced<noiseModel::Diagonal::shared_ptr>(diag3));
  EXPECT(equalsDereferencedXML<noiseModel::Diagonal::shared_ptr>(diag3));
  EXPECT(equalsDereferencedBinary<noiseModel::Diagonal::shared_ptr>(diag3));

  EXPECT(equalsDereferenced<noiseModel::Gaussian::shared_ptr>(gaussian3));
  EXPECT(equalsDereferencedXML<noiseModel::Gaussian::shared_ptr>(gaussian3));
  EXPECT(equalsDereferencedBinary<noiseModel::Gaussian::shared_ptr>(gaussian3));

  EXPECT(equalsDereferenced<noiseModel::Isotropic::shared_ptr>(iso3));
  EXPECT(equalsDereferencedXML<noiseModel::Isotropic::shared_ptr>(iso3));
  EXPECT(equalsDereferencedBinary<noiseModel::Isotropic::shared_ptr>(iso3));

  EXPECT(equalsDereferenced<noiseModel::Unit::shared_ptr>(unit3));
  EXPECT(equalsDereferencedXML<noiseModel::Unit::shared_ptr>(unit3));
  EXPECT(equalsDereferencedBinary<noiseModel::Unit::shared_ptr>(unit3));

  EXPECT(equalsDereferencedBinary<noiseModel::Constrained::shared_ptr>(constrained3));
  EXPECT(equalsDereferenced<noiseModel::Constrained::shared_ptr>(constrained3));
  EXPECT(equalsDereferencedXML<noiseModel::Constrained::shared_ptr>(constrained3));
}

/* ************************************************************************* */
TEST (Serialization, SharedNoiseModel_noiseModels) {
  SharedNoiseModel diag3_sg = diag3;
  EXPECT(equalsDereferenced<SharedNoiseModel>(diag3_sg));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(diag3_sg));
  EXPECT(equalsDereferencedBinary<SharedNoiseModel>(diag3_sg));

  EXPECT(equalsDereferenced<SharedNoiseModel>(diag3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(diag3));
  EXPECT(equalsDereferencedBinary<SharedNoiseModel>(diag3));

  EXPECT(equalsDereferenced<SharedNoiseModel>(iso3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(iso3));
  EXPECT(equalsDereferencedBinary<SharedNoiseModel>(iso3));

  EXPECT(equalsDereferenced<SharedNoiseModel>(gaussian3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(gaussian3));
  EXPECT(equalsDereferencedBinary<SharedNoiseModel>(gaussian3));

  EXPECT(equalsDereferenced<SharedNoiseModel>(unit3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(unit3));
  EXPECT(equalsDereferencedBinary<SharedNoiseModel>(unit3));

  EXPECT(equalsDereferencedBinary<SharedNoiseModel>(constrained3));
  EXPECT(equalsDereferenced<SharedNoiseModel>(constrained3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(constrained3));
}

/* ************************************************************************* */
TEST (Serialization, SharedDiagonal_noiseModels) {
  EXPECT(equalsDereferenced<SharedDiagonal>(diag3));
  EXPECT(equalsDereferencedXML<SharedDiagonal>(diag3));
  EXPECT(equalsDereferencedBinary<SharedDiagonal>(diag3));

  EXPECT(equalsDereferenced<SharedDiagonal>(iso3));
  EXPECT(equalsDereferencedXML<SharedDiagonal>(iso3));
  EXPECT(equalsDereferencedBinary<SharedDiagonal>(iso3));

  EXPECT(equalsDereferenced<SharedDiagonal>(unit3));
  EXPECT(equalsDereferencedXML<SharedDiagonal>(unit3));
  EXPECT(equalsDereferencedBinary<SharedDiagonal>(unit3));

  EXPECT(equalsDereferencedBinary<SharedDiagonal>(constrained3));
  EXPECT(equalsDereferenced<SharedDiagonal>(constrained3));
  EXPECT(equalsDereferencedXML<SharedDiagonal>(constrained3));
}

/* ************************************************************************* */
TEST(Serialization, graduatedLossFunctions) {
  using noiseModel::mEstimator::GemanMcClure;
  using noiseModel::mEstimator::TruncatedLeastSquares;

  const double k = 2.5;
  std::vector<noiseModel::mEstimator::Base::shared_ptr> losses{
      GemanMcClure::Create(k, GemanMcClure::GradScheme::STANDARD),
      GemanMcClure::Create(k, GemanMcClure::GradScheme::SCALE_INVARIANT),
      TruncatedLeastSquares::Create(
          k, TruncatedLeastSquares::GradScheme::STANDARD),
      TruncatedLeastSquares::Create(
          k, TruncatedLeastSquares::GradScheme::GNC_LINEAR),
      TruncatedLeastSquares::Create(
          k, TruncatedLeastSquares::GradScheme::GNC_SUPERLINEAR)};

  for (const auto& loss : losses) {
    EXPECT(equalsDereferenced<noiseModel::mEstimator::Base::shared_ptr>(loss));
    EXPECT(
        equalsDereferencedXML<noiseModel::mEstimator::Base::shared_ptr>(loss));
    EXPECT(equalsDereferencedBinary<noiseModel::mEstimator::Base::shared_ptr>(
        loss));
  }

  // Also verify the scheme survives on the concrete pointer types.
  auto gmc = GemanMcClure::Create(k, GemanMcClure::GradScheme::SCALE_INVARIANT);
  EXPECT(equalsDereferenced<GemanMcClure::shared_ptr>(gmc));
  EXPECT(equalsDereferencedXML<GemanMcClure::shared_ptr>(gmc));
  EXPECT(equalsDereferencedBinary<GemanMcClure::shared_ptr>(gmc));

  auto tls = TruncatedLeastSquares::Create(
      k, TruncatedLeastSquares::GradScheme::GNC_LINEAR);
  EXPECT(equalsDereferenced<TruncatedLeastSquares::shared_ptr>(tls));
  EXPECT(equalsDereferencedXML<TruncatedLeastSquares::shared_ptr>(tls));
  EXPECT(equalsDereferencedBinary<TruncatedLeastSquares::shared_ptr>(tls));
}

/* ************************************************************************* */
// equals() must distinguish two losses that differ only in graduation scheme.
TEST(Serialization, graduationSchemeInequality) {
  using noiseModel::mEstimator::GemanMcClure;
  using noiseModel::mEstimator::TruncatedLeastSquares;

  const double k = 2.5;
  auto gmcStandard =
      GemanMcClure::Create(k, GemanMcClure::GradScheme::STANDARD);
  auto gmcScaleInvariant =
      GemanMcClure::Create(k, GemanMcClure::GradScheme::SCALE_INVARIANT);
  EXPECT(gmcStandard->equals(*GemanMcClure::Create(k)));
  EXPECT(!gmcStandard->equals(*gmcScaleInvariant));
  EXPECT(!gmcScaleInvariant->equals(*gmcStandard));

  auto tlsStandard = TruncatedLeastSquares::Create(
      k, TruncatedLeastSquares::GradScheme::STANDARD);
  auto tlsGncLinear = TruncatedLeastSquares::Create(
      k, TruncatedLeastSquares::GradScheme::GNC_LINEAR);
  auto tlsGncSuperlinear = TruncatedLeastSquares::Create(
      k, TruncatedLeastSquares::GradScheme::GNC_SUPERLINEAR);
  EXPECT(tlsStandard->equals(*TruncatedLeastSquares::Create(k)));
  EXPECT(!tlsStandard->equals(*tlsGncLinear));
  EXPECT(!tlsGncLinear->equals(*tlsGncSuperlinear));
  EXPECT(!tlsGncSuperlinear->equals(*tlsStandard));
}

/* Create GUIDs for factors */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor")
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsam::HessianFactor")
BOOST_CLASS_EXPORT_GUID(gtsam::GaussianConditional , "gtsam::GaussianConditional")

/* ************************************************************************* */
TEST (Serialization, linear_factors) {
  VectorValues values;
  values.insert(0, Vector{{1.0}});
  values.insert(1, Vector2(2.0,3.0));
  values.insert(2, Vector2(4.0,5.0));
  EXPECT(equalsObj<VectorValues>(values));
  EXPECT(equalsXML<VectorValues>(values));
  EXPECT(equalsBinary<VectorValues>(values));

  Key i1 = 4, i2 = 7;
  Matrix A1 = I_3x3, A2 = -1.0 * I_3x3;
  Vector b = Vector::Ones(3);
  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector3(1.0, 2.0, 3.0));
  JacobianFactor jacobianfactor(i1, A1, i2, A2, b, model);
  EXPECT(equalsObj(jacobianfactor));
  EXPECT(equalsXML(jacobianfactor));
  EXPECT(equalsBinary(jacobianfactor));

  HessianFactor hessianfactor(jacobianfactor);
  EXPECT(equalsObj(hessianfactor));
  EXPECT(equalsXML(hessianfactor));
  EXPECT(equalsBinary(hessianfactor));
}

/* ************************************************************************* */
TEST (Serialization, gaussian_conditional) {
  Matrix A1{{1., 2.}, {3., 4.}};
  Matrix A2{{6., 0.2}, {8., 0.4}};
  Matrix R{{0.1, 0.3}, {0.0, 0.34}};
  Vector d{{0.2, 0.5}};
  GaussianConditional cg(0, d, R, 1, A1, 2, A2);

  EXPECT(equalsObj(cg));
  EXPECT(equalsXML(cg));
  EXPECT(equalsBinary(cg));
}

/* ************************************************************************* */
TEST (Serialization, gaussian_factor_graph) {
  GaussianFactorGraph graph;
  {
    Matrix A1{{1., 2.}, {3., 4.}};
    Matrix A2{{6., 0.2}, {8., 0.4}};
    Matrix R{{0.1, 0.3}, {0.0, 0.34}};
    Vector d{{0.2, 0.5}};
    GaussianConditional cg(0, d, R, 1, A1, 2, A2);
    graph.push_back(cg);
  }

  {
    Key i1 = 4, i2 = 7;
    Matrix A1 = I_3x3, A2 = -1.0 * I_3x3;
    Vector b = Vector::Ones(3);
    SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector3(1.0, 2.0, 3.0));
    JacobianFactor jacobianfactor(i1, A1, i2, A2, b, model);
    HessianFactor hessianfactor(jacobianfactor);
    graph.push_back(jacobianfactor);
    graph.push_back(hessianfactor);
  }
  EXPECT(equalsObj(graph));
  EXPECT(equalsXML(graph));
  EXPECT(equalsBinary(graph));
}

/* ****************************************************************************/
TEST(Serialization, gaussian_bayes_net) {
  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      0, Vector2(1.0, 2.0), Matrix2{{3.0, 4.0}, {0.0, 6.0}}, 3,
      Matrix2{{7.0, 8.0}, {9.0, 10.0}}, 4,
      Matrix2{{11.0, 12.0}, {13.0, 14.0}}));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      1, Vector2(15.0, 16.0), Matrix2{{17.0, 18.0}, {0.0, 20.0}}, 2,
      Matrix2{{21.0, 22.0}, {23.0, 24.0}}, 4,
      Matrix2{{25.0, 26.0}, {27.0, 28.0}}));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      2, Vector2(29.0, 30.0), Matrix2{{31.0, 32.0}, {0.0, 34.0}}, 3,
      Matrix2{{35.0, 36.0}, {37.0, 38.0}}));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      3, Vector2(39.0, 40.0), Matrix2{{41.0, 42.0}, {0.0, 44.0}}, 4,
      Matrix2{{45.0, 46.0}, {47.0, 48.0}}));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
      4, Vector2(49.0, 50.0), Matrix2{{51.0, 52.0}, {0.0, 54.0}}));

  std::string serialized = serialize(gbn);
  GaussianBayesNet actual;
  deserialize(serialized, actual);
  EXPECT(assert_equal(gbn, actual));
}

/* ************************************************************************* */
TEST (Serialization, gaussian_bayes_tree) {
  const Key x1=1, x2=2, x3=3, x4=4;
  const Ordering chainOrdering {x2, x1, x3, x4};
  const SharedDiagonal chainNoise = noiseModel::Isotropic::Sigma(1, 0.5);
  const GaussianFactorGraph chain = {
      std::make_shared<JacobianFactor>(x2, Matrix{{1.}}, x1, Matrix{{1.}},
                                       Vector{{1.}}, chainNoise),
      std::make_shared<JacobianFactor>(x2, Matrix{{1.}}, x3, Matrix{{1.}},
                                       Vector{{1.}}, chainNoise),
      std::make_shared<JacobianFactor>(x3, Matrix{{1.}}, x4, Matrix{{1.}},
                                       Vector{{1.}}, chainNoise),
      std::make_shared<JacobianFactor>(x4, Matrix{{1.}}, Vector{{1.}},
                                       chainNoise)};

  GaussianBayesTree init = *chain.eliminateMultifrontal(chainOrdering);
  GaussianBayesTree expected = *chain.eliminateMultifrontal(chainOrdering);
  GaussianBayesTree actual;

  std::string serialized = serialize(init);
  deserialize(serialized, actual);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
namespace bayes_tree_serialization {

using Clique = GaussianBayesTreeClique;

Clique::shared_ptr makeRoot(Key key) {
  return std::make_shared<Clique>(std::make_shared<GaussianConditional>(
      key, Vector{{0.0}}, Matrix{{1.0}}));
}

Clique::shared_ptr makeChild(Key key, Key parent) {
  return std::make_shared<Clique>(std::make_shared<GaussianConditional>(
      key, Vector{{0.0}}, Matrix{{1.0}}, parent, Matrix{{-1.0}}));
}

Key frontal(const Clique::shared_ptr& clique) {
  return clique->conditional()->frontals().front();
}

/** Serializes the recursive Bayes tree representation used by version 0. */
struct LegacyGaussianBayesTree {
  GaussianBayesTree::Nodes nodes;
  GaussianBayesTree::Roots roots;

  explicit LegacyGaussianBayesTree(const GaussianBayesTree& tree)
      : nodes(tree.nodes()), roots(tree.roots()) {}

  template <class Archive>
  void serialize(Archive& archive, const unsigned int /*version*/) {
    archive & boost::serialization::make_nvp("nodes_", nodes);
    archive & boost::serialization::make_nvp("roots_", roots);
  }
};

// Verifies that a deep chain round-trips without recursive serialization.
TEST(Serialization, LargeGaussianBayesTree) {
  constexpr size_t kCliqueCount = 10000;
  GaussianBayesTree input;
  Clique::shared_ptr parent = makeRoot(kCliqueCount - 1);
  input.insertRoot(parent);
  for (size_t key = kCliqueCount - 1; key-- > 0;) {
    Clique::shared_ptr child = makeChild(key, key + 1);
    input.addClique(child, parent);
    parent = child;
  }

  GaussianBayesTree actual;
  roundtripBinary(input, actual);

  EXPECT_LONGS_EQUAL(kCliqueCount, actual.nodes().size());
  EXPECT_LONGS_EQUAL(1, actual.roots().size());
  Clique::shared_ptr previous, clique = actual.roots().front();
  for (size_t key = kCliqueCount; key-- > 0;) {
    EXPECT_LONGS_EQUAL(key, frontal(clique));
    EXPECT(actual[key] == clique);
    EXPECT(clique->parent() == previous);
    EXPECT_LONGS_EQUAL(key == 0 ? 0 : 1, clique->nrChildren());
    previous = clique;
    if (key != 0) clique = clique->children.front();
  }
}

// Verifies that root and child ordering survive a flat round-trip.
TEST(Serialization, GaussianBayesTreeStructure) {
  GaussianBayesTree input;
  Clique::shared_ptr firstRoot = makeRoot(10), secondRoot = makeRoot(20);
  input.insertRoot(firstRoot);
  input.addClique(makeChild(11, 10), firstRoot);
  input.addClique(makeChild(12, 10), firstRoot);
  input.insertRoot(secondRoot);
  input.addClique(makeChild(21, 20), secondRoot);

  GaussianBayesTree actual;
  roundtripBinary(input, actual);

  EXPECT_LONGS_EQUAL(2, actual.roots().size());
  EXPECT_LONGS_EQUAL(10, frontal(actual.roots()[0]));
  EXPECT_LONGS_EQUAL(20, frontal(actual.roots()[1]));
  EXPECT_LONGS_EQUAL(11, frontal(actual.roots()[0]->children[0]));
  EXPECT_LONGS_EQUAL(12, frontal(actual.roots()[0]->children[1]));
  EXPECT_LONGS_EQUAL(21, frontal(actual.roots()[1]->children[0]));
}

// Verifies loading the recursive format written before BayesTree version 1.
TEST(Serialization, LegacyGaussianBayesTree) {
  GaussianBayesTree input;
  input.insertRoot(makeRoot(7));

  LegacyGaussianBayesTree legacy(input);
  stringstream stream;
  {
    boost::archive::text_oarchive archive(stream);
    archive << legacy;
  }

  GaussianBayesTree actual;
  {
    boost::archive::text_iarchive archive(stream);
    archive >> actual;
  }

  EXPECT_LONGS_EQUAL(1, actual.nodes().size());
  EXPECT_LONGS_EQUAL(1, actual.roots().size());
  EXPECT_LONGS_EQUAL(7, frontal(actual.roots().front()));
}

}  // namespace bayes_tree_serialization
/* ************************************************************************* */

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
