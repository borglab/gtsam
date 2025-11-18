//*************************************************************************
// hybrid
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3f.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <gtsam/hybrid/HybridValues.h>
class HybridValues {
  gtsam::VectorValues continuous() const;
  gtsam::DiscreteValues discrete() const;
  gtsam::Values& nonlinear() const;

  HybridValues();
  HybridValues(const gtsam::VectorValues& cv, const gtsam::DiscreteValues& dv);
  HybridValues(const gtsam::VectorValues& cv, const gtsam::DiscreteValues& dv, const gtsam::Values& v);

  void print(string s = "HybridValues",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridValues& other, double tol) const;

  void insert(gtsam::Key j, int value);
  void insert(gtsam::Key j, const gtsam::Vector& value);
  void insert_or_assign(gtsam::Key j, const gtsam::Vector& value);
  void insert_or_assign(gtsam::Key j, size_t value);
  
  // Use same (important) order as in values.i
  void insertNonlinear(gtsam::Key j, gtsam::Vector vector);
  void insertNonlinear(gtsam::Key j, gtsam::Matrix matrix);
  void insertNonlinear(gtsam::Key j, const gtsam::Point2& point2);
  void insertNonlinear(gtsam::Key j, const gtsam::Point3& point3);
  void insertNonlinear(gtsam::Key j, const gtsam::Rot2& rot2);
  void insertNonlinear(gtsam::Key j, const gtsam::Pose2& pose2);
  void insertNonlinear(gtsam::Key j, const gtsam::SO3& R);
  void insertNonlinear(gtsam::Key j, const gtsam::SO4& Q);
  void insertNonlinear(gtsam::Key j, const gtsam::SOn& P);
  void insertNonlinear(gtsam::Key j, const gtsam::Rot3& rot3);
  void insertNonlinear(gtsam::Key j, const gtsam::Pose3& pose3);
  void insertNonlinear(gtsam::Key j, const gtsam::Similarity2& similarity2);
  void insertNonlinear(gtsam::Key j, const gtsam::Similarity3& similarity3);
  void insertNonlinear(gtsam::Key j, const gtsam::Unit3& unit3);
  void insertNonlinear(gtsam::Key j, const gtsam::Cal3Bundler& cal3bundler);
  void insertNonlinear(gtsam::Key j, const gtsam::Cal3f& cal3f);
  void insertNonlinear(gtsam::Key j, const gtsam::Cal3_S2& cal3_s2);
  void insertNonlinear(gtsam::Key j, const gtsam::Cal3DS2& cal3ds2);
  void insertNonlinear(gtsam::Key j, const gtsam::Cal3Fisheye& cal3fisheye);
  void insertNonlinear(gtsam::Key j, const gtsam::Cal3Unified& cal3unified);
  void insertNonlinear(gtsam::Key j, const gtsam::EssentialMatrix& E);
  void insertNonlinear(gtsam::Key j, const gtsam::FundamentalMatrix& F);
  void insertNonlinear(gtsam::Key j, const gtsam::SimpleFundamentalMatrix& F);
  void insertNonlinear(gtsam::Key j, const gtsam::OrientedPlane3& plane);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3f>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3DS2>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3f>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3DS2>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void insertNonlinear(gtsam::Key j, const gtsam::imuBias::ConstantBias& constant_bias);
  void insertNonlinear(gtsam::Key j, const gtsam::NavState& nav_state);
  void insertNonlinear(gtsam::Key j, double c);

  void insert(const gtsam::VectorValues& values);
  void insert(const gtsam::DiscreteValues& values);
  void insert(const gtsam::Values& values);
  void insert(const gtsam::HybridValues& values);


  void update(const gtsam::VectorValues& values);
  void update(const gtsam::DiscreteValues& values);
  void update(const gtsam::Values& values);
  void update(const gtsam::HybridValues& values);

  bool existsVector(gtsam::Key j);
  bool existsDiscrete(gtsam::Key j);
  bool existsNonlinear(gtsam::Key j);
  bool exists(gtsam::Key j);

  gtsam::HybridValues retract(const gtsam::VectorValues& delta) const;

  size_t& atDiscrete(gtsam::Key j);
  gtsam::Vector& at(gtsam::Key j);
};

#include <gtsam/hybrid/HybridFactor.h>
class AlgebraicDecisionTreeKey {
  const double& operator()(const gtsam::DiscreteValues& x) const;
  void print(string s = "AlgebraicDecisionTreeKey\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};
virtual class HybridFactor : gtsam::Factor {
  void print(string s = "HybridFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridFactor& lf, double tol = 1e-9) const;

  // Standard interface:
  bool isDiscrete() const;
  bool isContinuous() const;
  bool isHybrid() const;
  size_t nrContinuous() const;
  gtsam::DiscreteKeys discreteKeys() const;
  gtsam::KeyVector continuousKeys() const;
  double error(const gtsam::HybridValues& hybridValues) const;
  gtsam::AlgebraicDecisionTreeKey errorTree(const gtsam::VectorValues &continuousValues);
  gtsam::Factor restrict(const gtsam::DiscreteValues& assignment) const;
};

#include <gtsam/hybrid/HybridConditional.h>
virtual class HybridConditional : gtsam::HybridFactor {
  HybridConditional();
  HybridConditional(const gtsam::KeyVector& continuousKeys, 
                    const gtsam::DiscreteKeys& discreteKeys, size_t nFrontals);
  HybridConditional(const gtsam::KeyVector& continuousFrontals,
                    const gtsam::DiscreteKeys& discreteFrontals,
                    const gtsam::KeyVector& continuousParents,
                    const gtsam::DiscreteKeys& discreteParents);
  HybridConditional(const gtsam::GaussianConditional::shared_ptr& continuousConditional);
  HybridConditional(const gtsam::DiscreteConditional::shared_ptr& discreteConditional);
  HybridConditional(const gtsam::HybridGaussianConditional::shared_ptr& hybridGaussianCond);

  void print(string s = "Hybrid Conditional\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridConditional& other, double tol = 1e-9) const;
  size_t nrFrontals() const;
  size_t nrParents() const;

  // Standard interface:
  double negLogConstant() const;
  double logProbability(const gtsam::HybridValues& values) const;
  double evaluate(const gtsam::HybridValues& values) const;
  double operator()(const gtsam::HybridValues& values) const;

  bool isDiscrete() const;
  bool isContinuous() const;
  bool isHybrid() const;
  gtsam::HybridGaussianConditional* asHybrid() const;
  gtsam::GaussianConditional* asGaussian() const;
  gtsam::DiscreteConditional* asDiscrete() const;

  gtsam::Factor* inner();
};

#include <gtsam/hybrid/HybridGaussianFactor.h>
class HybridGaussianFactor : gtsam::HybridFactor {
  HybridGaussianFactor(
      const gtsam::DiscreteKey& discreteKey,
      const std::vector<gtsam::GaussianFactor::shared_ptr>& factors);
  HybridGaussianFactor(
      const gtsam::DiscreteKey& discreteKey,
      const std::vector<std::pair<gtsam::GaussianFactor::shared_ptr, double>>&
          factorPairs);

  void print(string s = "HybridGaussianFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/HybridGaussianConditional.h>
class HybridGaussianConditional : gtsam::HybridGaussianFactor {
  HybridGaussianConditional(
      const gtsam::DiscreteKeys& discreteParents,
      const gtsam::HybridGaussianConditional::Conditionals& conditionals);
  HybridGaussianConditional(
      const gtsam::DiscreteKey& discreteParent,
      const std::vector<gtsam::GaussianConditional::shared_ptr>& conditionals);
  // Standard API
  gtsam::GaussianConditional::shared_ptr choose(
      const gtsam::DiscreteValues &discreteValues) const;
//   gtsam::GaussianConditional::shared_ptr operator()(
//       const gtsam::DiscreteValues &discreteValues) const;
  size_t nrComponents() const;
  gtsam::KeyVector continuousParents() const;
  double negLogConstant() const;

  gtsam::HybridGaussianFactor* likelihood(
      const gtsam::VectorValues& frontals) const;
  double logProbability(const gtsam::HybridValues& values) const;
  double evaluate(const gtsam::HybridValues& values) const;

  HybridGaussianConditional::shared_ptr prune(
      const gtsam::DiscreteConditional &discreteProbs) const;
  bool pruned() const;

  void print(string s = "HybridGaussianConditional\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/HybridBayesTree.h>
class HybridBayesTreeClique {
  HybridBayesTreeClique();
  HybridBayesTreeClique(const gtsam::HybridConditional* conditional);
  const gtsam::HybridConditional* conditional() const;
  bool isRoot() const;
  // double evaluate(const gtsam::HybridValues& values) const;
};

class HybridBayesTree {
  HybridBayesTree();
  void print(string s = "HybridBayesTree\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridBayesTree& other, double tol = 1e-9) const;

  size_t size() const;
  bool empty() const;
  const HybridBayesTreeClique* operator[](size_t j) const;

  gtsam::HybridValues optimize() const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/HybridBayesNet.h>
class HybridBayesNet {
  HybridBayesNet();
  void push_back(const gtsam::HybridGaussianConditional* s);
  void push_back(const gtsam::GaussianConditional* s);
  void push_back(const gtsam::DiscreteConditional* s);
  void push_back(gtsam::HybridConditional::shared_ptr conditional);

  bool empty() const;
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::HybridConditional* at(size_t i) const;

  // Standard interface:
  double logProbability(const gtsam::HybridValues& x) const;
  double evaluate(const gtsam::HybridValues& values) const;
  double error(const gtsam::HybridValues& values) const;
  gtsam::AlgebraicDecisionTreeKey errorTree(
      const gtsam::VectorValues& continuousValues) const;

  gtsam::HybridGaussianFactorGraph toFactorGraph(
      const gtsam::VectorValues& measurements) const;

  double negLogConstant() const;
  double negLogConstant(const gtsam::DiscreteValues &discrete) const;
  gtsam::AlgebraicDecisionTreeKey discretePosterior(
      const gtsam::VectorValues &continuousValues) const;
  gtsam::DiscreteBayesNet discreteMarginal() const;
  gtsam::GaussianBayesNet choose(const gtsam::DiscreteValues& assignment) const;

  gtsam::DiscreteBayesNet discreteMarginal() const;
  gtsam::DiscreteValues mpe() const;

  gtsam::HybridValues optimize() const;
  gtsam::VectorValues optimize(const gtsam::DiscreteValues& assignment) const;

  gtsam::HybridValues sample(const gtsam::HybridValues& given, std::mt19937_64@ rng = nullptr) const;
  gtsam::HybridValues sample(std::mt19937_64@ rng = nullptr) const;

  gtsam::HybridBayesNet prune(size_t maxNrLeaves) const;
  gtsam::HybridBayesNet prune(size_t maxNrLeaves, double marginalThreshold) const;
  // gtsam::HybridBayesNet prune(size_t maxNrLeaves,
  //   const std::optional<double> &marginalThreshold = std::nullopt,
  //   gtsam::DiscreteValues *fixedValues = nullptr) const;

  void print(string s = "HybridBayesNet\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridBayesNet& fg, double tol = 1e-9) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      string s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
};

#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
class HybridGaussianFactorGraph {
  HybridGaussianFactorGraph();
  HybridGaussianFactorGraph(const gtsam::HybridBayesNet& bayesNet);

  // Building the graph
  void push_back(const gtsam::HybridFactor* factor);
  void push_back(const gtsam::HybridConditional* conditional);
  void push_back(const gtsam::HybridGaussianFactorGraph& graph);
  void push_back(const gtsam::HybridBayesNet& bayesNet);
  void push_back(const gtsam::HybridBayesTree& bayesTree);
  void push_back(const gtsam::HybridGaussianFactor* gmm);
  void push_back(gtsam::DecisionTreeFactor* factor);
  void push_back(gtsam::TableFactor* factor);
  void push_back(gtsam::JacobianFactor* factor);

  bool empty() const;
  void remove(size_t i);
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::HybridFactor* at(size_t i) const;

  void print(string s = "") const;
  bool equals(const gtsam::HybridGaussianFactorGraph& fg,
              double tol = 1e-9) const;

  // evaluation
  double error(const gtsam::HybridValues& values) const;
  double probPrime(const gtsam::HybridValues& values) const;

  gtsam::HybridBayesNet* eliminateSequential();
  gtsam::HybridBayesNet* eliminateSequential(
      gtsam::Ordering::OrderingType type);
  gtsam::HybridBayesNet* eliminateSequential(const gtsam::Ordering& ordering);
  pair<gtsam::HybridBayesNet*, gtsam::HybridGaussianFactorGraph*>
  eliminatePartialSequential(const gtsam::Ordering& ordering);

  gtsam::HybridBayesTree* eliminateMultifrontal();
  gtsam::HybridBayesTree* eliminateMultifrontal(
      gtsam::Ordering::OrderingType type);
  gtsam::HybridBayesTree* eliminateMultifrontal(
      const gtsam::Ordering& ordering);
  pair<gtsam::HybridBayesTree*, gtsam::HybridGaussianFactorGraph*>
  eliminatePartialMultifrontal(const gtsam::Ordering& ordering);

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
};
const gtsam::Ordering HybridOrdering(const gtsam::HybridGaussianFactorGraph& graph);

#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
class HybridNonlinearFactorGraph {
  HybridNonlinearFactorGraph();
  HybridNonlinearFactorGraph(const gtsam::HybridNonlinearFactorGraph& graph);
  void push_back(gtsam::HybridFactor* factor);
  void push_back(gtsam::NonlinearFactor* factor);
  void push_back(gtsam::DiscreteFactor* factor);
  void push_back(const gtsam::HybridNonlinearFactorGraph& graph);
  // TODO(Varun) Wrap add() methods

  gtsam::HybridGaussianFactorGraph linearize(
      const gtsam::Values& continuousValues) const;

  bool empty() const;
  void remove(size_t i);
  size_t size() const;
  void resize(size_t size);
  gtsam::KeySet keys() const;
  const gtsam::HybridFactor* at(size_t i) const;
  gtsam::HybridNonlinearFactorGraph restrict(
      const gtsam::DiscreteValues& assignment) const;

  void print(string s = "HybridNonlinearFactorGraph\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/HybridNonlinearFactor.h>
class HybridNonlinearFactor : gtsam::HybridFactor {
  HybridNonlinearFactor(const gtsam::DiscreteKey& discreteKey,
                        const std::vector<gtsam::NoiseModelFactor*>& factors);

  HybridNonlinearFactor(
      const gtsam::DiscreteKey& discreteKey,
      const std::vector<std::pair<gtsam::NoiseModelFactor*, double>>& factors);

  HybridNonlinearFactor(
      const gtsam::DiscreteKeys& discreteKeys,
      const gtsam::DecisionTree<
          gtsam::Key, std::pair<gtsam::NoiseModelFactor*, double>>& factors);

  double error(const gtsam::Values& continuousValues,
               const gtsam::DiscreteValues& discreteValues) const;

  HybridGaussianFactor* linearize(const gtsam::Values& continuousValues) const;

  void print(string s = "HybridNonlinearFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/HybridSmoother.h>
class HybridSmoother {
  HybridSmoother(const std::optional<double> marginalThreshold = std::nullopt);

  const gtsam::DiscreteValues& fixedValues() const;
  void reInitialize(gtsam::HybridBayesNet& hybridBayesNet);

  void update(
      const gtsam::HybridNonlinearFactorGraph& graph,
      const gtsam::Values& initial,
      std::optional<size_t> maxNrLeaves = std::nullopt,
      const std::optional<gtsam::Ordering> given_ordering = std::nullopt);

  void relinearize(
      const std::optional<gtsam::Ordering> givenOrdering = std::nullopt);

  gtsam::Values linearizationPoint() const;
  gtsam::HybridNonlinearFactorGraph allFactors() const;

  gtsam::Ordering getOrdering(const gtsam::HybridGaussianFactorGraph& factors,
                              const gtsam::KeySet& newFactorKeys);

  std::pair<gtsam::HybridGaussianFactorGraph, gtsam::HybridBayesNet>
  addConditionals(const gtsam::HybridGaussianFactorGraph& graph,
                  const gtsam::HybridBayesNet& hybridBayesNet) const;

  gtsam::HybridGaussianConditional* gaussianMixture(size_t index) const;

  const gtsam::HybridBayesNet& hybridBayesNet() const;
  gtsam::HybridValues optimize() const;
};

}  // namespace gtsam
