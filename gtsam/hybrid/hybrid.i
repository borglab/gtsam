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
  const gtsam::VectorValues& continuous() const;
  const gtsam::DiscreteValues& discrete() const;
  const gtsam::Values& nonlinear() const;

  HybridValues();
  HybridValues(const gtsam::VectorValues& cv, const gtsam::DiscreteValues& dv);
  HybridValues(const gtsam::VectorValues& cv, const gtsam::DiscreteValues& dv, const gtsam::Values& v);
  HybridValues(const gtsam::DiscreteValues& dv, const gtsam::Values& v);

  void print(string s = "HybridValues",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridValues& other, double tol) const;

  gtsam::HybridValues& insert(gtsam::Key j, size_t value);
  gtsam::HybridValues& insert(gtsam::Key j, const gtsam::Vector& value);
  void insert_or_assign(gtsam::Key j, const gtsam::Vector& value);
  void insert_or_assign(gtsam::Key j, size_t value);
  
  // Use same (important) order as in values.i
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Vector& vector);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Matrix& matrix);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Point2& point2);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Point3& point3);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Rot2& rot2);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Pose2& pose2);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::SO3& R);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::SO4& Q);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::SOn& P);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Rot3& rot3);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Pose3& pose3);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Similarity2& similarity2);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Similarity3& similarity3);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Unit3& unit3);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Cal3Bundler& cal3bundler);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Cal3f& cal3f);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Cal3_S2& cal3_s2);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Cal3DS2& cal3ds2);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Cal3Fisheye& cal3fisheye);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::Cal3Unified& cal3unified);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::EssentialMatrix& E);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::FundamentalMatrix& F);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::SimpleFundamentalMatrix& F);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::OrientedPlane3& plane);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3f>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3DS2>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3f>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3DS2>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::imuBias::ConstantBias& constant_bias);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const gtsam::NavState& nav_state);
  gtsam::HybridValues& insertNonlinear(gtsam::Key j, const double& c);

  gtsam::HybridValues& insert(const gtsam::VectorValues& values);
  gtsam::HybridValues& insert(const gtsam::DiscreteValues& values);
  gtsam::HybridValues& insert(const gtsam::Values& values);
  gtsam::HybridValues& insert(const gtsam::HybridValues& values);


  gtsam::HybridValues& update(const gtsam::VectorValues& values);
  gtsam::HybridValues& update(const gtsam::DiscreteValues& values);
  gtsam::HybridValues& update(const gtsam::Values& values);
  gtsam::HybridValues& update(const gtsam::HybridValues& values);

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
  bool equals(const gtsam::HybridFactor& lf, double tol = 1e-9) const;

  // Standard interface:
  bool isDiscrete() const;
  bool isContinuous() const;
  bool isHybrid() const;
  size_t nrContinuous() const;
  const gtsam::DiscreteKeys& discreteKeys() const;
  const gtsam::KeyVector& continuousKeys() const;
  double error(const gtsam::HybridValues& hybridValues) const;
  gtsam::AlgebraicDecisionTreeKey errorTree(
      const gtsam::VectorValues& continuousValues) const;
  std::shared_ptr<gtsam::Factor> restrict(
      const gtsam::DiscreteValues& discreteValues) const;
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
  HybridConditional(const gtsam::GaussianConditional* continuousConditional);
  HybridConditional(const gtsam::DiscreteConditional* discreteConditional);
  HybridConditional(const gtsam::HybridGaussianConditional* hybridGaussianCond);

  size_t nrFrontals() const;
  size_t nrParents() const;

  // Standard interface:
  double negLogConstant() const;
  double logProbability(const gtsam::HybridValues& values) const;
  double evaluate(const gtsam::HybridValues& values) const;
//   double operator()(const gtsam::HybridValues& values) const;

  gtsam::HybridGaussianConditional* asHybrid() const;
  gtsam::GaussianConditional* asGaussian() const;
  gtsam::DiscreteConditional* asDiscrete() const;

  std::shared_ptr<gtsam::Factor> inner() const;
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
  std::pair<gtsam::GaussianFactor::shared_ptr, double> operator()(
      const gtsam::DiscreteValues& assignment) const;

};

#include <gtsam/hybrid/HybridGaussianConditional.h>
class HybridGaussianConditional : gtsam::HybridGaussianFactor {
  HybridGaussianConditional(
      const gtsam::DiscreteKeys& discreteParents,
      const gtsam::HybridGaussianConditional::Conditionals& conditionals);
  HybridGaussianConditional(
      const gtsam::DiscreteKey& discreteParent,
      const std::vector<gtsam::GaussianConditional::shared_ptr>& conditionals);
  HybridGaussianConditional(
      const gtsam::DiscreteKey& discreteParent, gtsam::Key key,
      const gtsam::Matrix& A, gtsam::Key parent,
      const std::vector<std::pair<gtsam::Vector, double>>& parameters);
  HybridGaussianConditional(
      const gtsam::DiscreteKey& discreteParent, gtsam::Key key,  //
      const gtsam::Matrix& A1, gtsam::Key parent1, const gtsam::Matrix& A2,
      gtsam::Key parent2,
      const std::vector<std::pair<gtsam::Vector, double>>& parameters);

  // Standard API
  gtsam::GaussianConditional* choose(
      const gtsam::DiscreteValues &discreteValues) const;
//   gtsam::GaussianConditional* operator()(
//       const gtsam::DiscreteValues &discreteValues) const;
  size_t nrComponents() const;
  gtsam::KeyVector continuousParents() const;
  double negLogConstant() const;

  gtsam::HybridGaussianFactor* likelihood(
      const gtsam::VectorValues& frontals) const;
  double logProbability(const gtsam::HybridValues& values) const;
  double evaluate(const gtsam::HybridValues& values) const;
//   double operator()(const gtsam::HybridValues &values) const;

  gtsam::HybridGaussianConditional::shared_ptr prune(
      const gtsam::DiscreteConditional &discreteProbs) const;
  bool pruned() const;

};

#include <gtsam/hybrid/HybridBayesTree.h>
class HybridBayesTreeClique {
  HybridBayesTreeClique();
  HybridBayesTreeClique(const gtsam::HybridConditional* conditional);
  const gtsam::HybridConditional::shared_ptr& conditional() const;
  bool isRoot() const;
  // double evaluate(const gtsam::HybridValues& values) const;
};

virtual class HybridBayesTree {
  HybridBayesTree();
  void print(string s = "HybridBayesTree\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridBayesTree& other, double tol = 1e-9) const;

  size_t size() const;
  bool empty() const;
  const HybridBayesTreeClique* operator[](gtsam::Key j) const;

  gtsam::HybridValues optimize() const;
  gtsam::VectorValues optimize(const gtsam::DiscreteValues& assignment) const;

  gtsam::GaussianBayesTree choose(const gtsam::DiscreteValues& assignment) const;
  double error(const gtsam::HybridValues& values) const;
  gtsam::DiscreteValues mpe() const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/HybridBayesNet.h>
class HybridBayesNet {
  HybridBayesNet();
  void push_back(
      const std::shared_ptr<gtsam::HybridGaussianConditional>& conditional);
  void push_back(
      const std::shared_ptr<gtsam::GaussianConditional>& conditional);
  void push_back(
      const std::shared_ptr<gtsam::DiscreteConditional>& conditional);
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

  double negLogConstant(
      const std::optional<gtsam::DiscreteValues>& discrete = std::nullopt) const;
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

  gtsam::HybridBayesNet prune(
      size_t maxNrLeaves,
      const std::optional<double>& marginalThreshold = std::nullopt,
      gtsam::DiscreteValues@ fixedValues = nullptr) const;

  void print(string s = "HybridBayesNet\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridBayesNet& fg, double tol = 1e-9) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      const string& s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
};

#include <gtsam/inference/FactorGraph.h>
template<FACTOR = {gtsam::Factor}>
virtual class FactorGraph {
  bool equals(const gtsam::FactorGraph<gtsam::Factor>& fg,
              double tol = 1e-9) const;
};

#include <gtsam/hybrid/HybridFactorGraph.h>
virtual class HybridFactorGraph : gtsam::FactorGraph<gtsam::Factor> {
  HybridFactorGraph();
  gtsam::KeySet keys() const;
  gtsam::KeySet discreteKeySet() const;
  const gtsam::KeySet continuousKeySet() const;

  // Building the graph
  void push_back(const gtsam::HybridFactor* factor);
  void push_back(const gtsam::HybridConditional* conditional);
  void push_back(const gtsam::HybridGaussianFactorGraph& graph);
  void push_back(const gtsam::HybridBayesNet& bayesNet);
  @pybind_lambda
  void push_back(const gtsam::HybridBayesTree& bayesTree);
  void push_back(const gtsam::HybridGaussianFactor* gmm);
  void push_back(gtsam::DecisionTreeFactor* factor);
  void push_back(gtsam::TableFactor* factor);
  void push_back(gtsam::JacobianFactor* factor);
  void push_back(gtsam::NonlinearFactor* factor);
  void push_back(gtsam::DiscreteFactor* factor);
  void push_back(const gtsam::HybridNonlinearFactorGraph& graph);
  
  void add(const gtsam::HybridFactor* factor);
  void add(const gtsam::HybridConditional* conditional);
  void add(const gtsam::HybridGaussianFactorGraph& graph);
  void add(const gtsam::HybridBayesNet& bayesNet);
  void add(const gtsam::HybridBayesTree& bayesTree);
  void add(const gtsam::HybridGaussianFactor* gmm);
  void add(gtsam::DecisionTreeFactor* factor);
  void add(gtsam::TableFactor* factor);
  void add(gtsam::JacobianFactor* factor);
  void add(gtsam::NonlinearFactor* factor);
  void add(gtsam::DiscreteFactor* factor);
  void add(const gtsam::HybridNonlinearFactorGraph& graph);
  
  bool empty() const;
  void remove(size_t i);
  size_t size() const;
  void resize(size_t size);
  const gtsam::HybridFactor* at(size_t i) const;

  // evaluation
  double error(const gtsam::HybridValues& values) const;

  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
};

#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
virtual class HybridGaussianFactorGraph : gtsam::HybridFactorGraph {
  HybridGaussianFactorGraph();
  HybridGaussianFactorGraph(const gtsam::HybridBayesNet& bayesNet);

  void printErrors(const gtsam::HybridValues& values,
                   const string& str = "HybridGaussianFactorGraph: ",
                   const gtsam::KeyFormatter& keyFormatter =
                       gtsam::DefaultKeyFormatter,
                   const gtsam::FactorErrorPredicate& printCondition =
                       gtsam::FactorErrorPredicate{
                           [](const gtsam::Factor*, double, size_t) {
                             return true;
                           }})
      const;

  gtsam::AlgebraicDecisionTreeKey errorTree(
      const gtsam::VectorValues& continuousValues) const;
  double probPrime(const gtsam::HybridValues& values) const;
  gtsam::AlgebraicDecisionTreeKey discretePosterior(
      const gtsam::VectorValues& continuousValues) const;

  // Sequential Elimination
  std::shared_ptr<gtsam::HybridBayesNet> eliminateSequential(
      gtsam::HybridGaussianFactorGraph::OptionalOrderingType orderingType = std::nullopt,
      const gtsam::HybridGaussianFactorGraph::Eliminate& function =
          gtsam::HybridGaussianFactorGraph::Eliminate(
              gtsam::HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::HybridGaussianFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  std::shared_ptr<gtsam::HybridBayesNet> eliminateSequential(
      const gtsam::Ordering& ordering,
      const gtsam::HybridGaussianFactorGraph::Eliminate& function =
          gtsam::HybridGaussianFactorGraph::Eliminate(
              gtsam::HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::HybridGaussianFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  pair<std::shared_ptr<gtsam::HybridBayesNet>,
       std::shared_ptr<gtsam::HybridGaussianFactorGraph>>
  eliminatePartialSequential(
      const gtsam::Ordering& ordering,
      const gtsam::HybridGaussianFactorGraph::Eliminate& function =
          gtsam::HybridGaussianFactorGraph::Eliminate(
              gtsam::HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::HybridGaussianFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  pair<std::shared_ptr<gtsam::HybridBayesNet>,
       std::shared_ptr<gtsam::HybridGaussianFactorGraph>>
  eliminatePartialSequential(
      const gtsam::KeyVector& variables,
      const gtsam::HybridGaussianFactorGraph::Eliminate& function =
          gtsam::HybridGaussianFactorGraph::Eliminate(
              gtsam::HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::HybridGaussianFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;

  // Multifrontal Elimination
  std::shared_ptr<gtsam::HybridBayesTree> eliminateMultifrontal(
      gtsam::HybridGaussianFactorGraph::OptionalOrderingType orderingType = std::nullopt,
      const gtsam::HybridGaussianFactorGraph::Eliminate& function =
          gtsam::HybridGaussianFactorGraph::Eliminate(
              gtsam::HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::HybridGaussianFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  std::shared_ptr<gtsam::HybridBayesTree> eliminateMultifrontal(
      const gtsam::Ordering& ordering,
      const gtsam::HybridGaussianFactorGraph::Eliminate& function =
          gtsam::HybridGaussianFactorGraph::Eliminate(
              gtsam::HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::HybridGaussianFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  pair<std::shared_ptr<gtsam::HybridBayesTree>,
       std::shared_ptr<gtsam::HybridGaussianFactorGraph>>
  eliminatePartialMultifrontal(
      const gtsam::Ordering& ordering,
      const gtsam::HybridGaussianFactorGraph::Eliminate& function =
          gtsam::HybridGaussianFactorGraph::Eliminate(
              gtsam::HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::HybridGaussianFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  pair<std::shared_ptr<gtsam::HybridBayesTree>,
       std::shared_ptr<gtsam::HybridGaussianFactorGraph>>
  eliminatePartialMultifrontal(
      const gtsam::KeyVector& variables,
      const gtsam::HybridGaussianFactorGraph::Eliminate& function =
          gtsam::HybridGaussianFactorGraph::Eliminate(
              gtsam::HybridGaussianFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::HybridGaussianFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;

  gtsam::GaussianFactorGraph choose(const gtsam::DiscreteValues& assignment) const;
  gtsam::GaussianFactorGraph operator()(const gtsam::DiscreteValues& assignment) const;

  gtsam::DiscreteFactorGraph discreteFactors() const;
};
const gtsam::Ordering HybridOrdering(const gtsam::HybridGaussianFactorGraph& graph);

#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
virtual class HybridNonlinearFactorGraph : gtsam::HybridFactorGraph {
  HybridNonlinearFactorGraph();
  HybridNonlinearFactorGraph(const gtsam::HybridNonlinearFactorGraph& graph);

  void printErrors(const gtsam::HybridValues& values,
                   const string& str = "HybridNonlinearFactorGraph: ",
                   const gtsam::KeyFormatter& keyFormatter =
                       gtsam::DefaultKeyFormatter,
                   const gtsam::FactorErrorPredicate& printCondition =
                       gtsam::FactorErrorPredicate{
                           [](const gtsam::Factor*, double, size_t) {
                             return true;
                           }})
      const;

  gtsam::AlgebraicDecisionTreeKey errorTree(const gtsam::Values& continuousValues) const;

  std::shared_ptr<gtsam::HybridGaussianFactorGraph> linearize(
      const gtsam::Values& continuousValues) const;

  gtsam::AlgebraicDecisionTreeKey discretePosterior(
      const gtsam::Values& continuousValues) const;

  gtsam::HybridNonlinearFactorGraph restrict(
      const gtsam::DiscreteValues& assignment) const;

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
               const gtsam::DiscreteValues& assignment) const;
  gtsam::AlgebraicDecisionTreeKey errorTree(
      const gtsam::Values& continuousValues) const;

  std::shared_ptr<gtsam::HybridGaussianFactor> linearize(
      const gtsam::Values& continuousValues) const;

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

#include <gtsam/hybrid/HybridGaussianISAM.h>
virtual class HybridGaussianISAM : gtsam::HybridBayesTree {
  HybridGaussianISAM();
  HybridGaussianISAM(const gtsam::HybridBayesTree& bayesTree);
  void update(
      const gtsam::HybridGaussianFactorGraph& newFactors,
      const std::optional<size_t>& maxNrLeaves = std::nullopt,
      const std::optional<gtsam::Ordering>& ordering = std::nullopt,
      const gtsam::HybridBayesTree::Eliminate& function =
          gtsam::HybridBayesTree::Eliminate(
              gtsam::HybridBayesTree::EliminationTraitsType::DefaultEliminate));
  static gtsam::Ordering GetOrdering(gtsam::HybridGaussianFactorGraph& factors,
                              const gtsam::HybridGaussianFactorGraph& newFactors);
};

#include <gtsam/hybrid/HybridNonlinearISAM.h>
class HybridNonlinearISAM {
  HybridNonlinearISAM(int reorderInterval = 1);

  // Standard Interface
  gtsam::Values estimate();
  const gtsam::HybridGaussianISAM& bayesTree() const;
  void prune(const size_t maxNumberLeaves);
  const gtsam::Values& getLinearizationPoint() const;
  const gtsam::DiscreteValues& assignment() const;
  int reorderInterval() const;
  int reorderCounter() const;
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void printStats() const;
  void saveGraph(const std::string& s,
                 const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void update(const gtsam::HybridNonlinearFactorGraph& newFactors,
              const gtsam::Values& initialValues,
              const std::optional<size_t>& maxNrLeaves = std::nullopt,
              const std::optional<gtsam::Ordering>& ordering = std::nullopt);
  void reorderRelinearize();
};

#include <gtsam/hybrid/HybridEliminationTree.h>
class HybridEliminationTree {
  HybridEliminationTree(const gtsam::HybridGaussianFactorGraph& factorGraph,
                        const gtsam::VariableIndex& structure, const gtsam::Ordering& order);
  HybridEliminationTree(const gtsam::HybridGaussianFactorGraph& factorGraph,
                        const gtsam::Ordering& order);
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridEliminationTree& other, double tol = 1e-9) const;
};

#include <gtsam/hybrid/HybridJunctionTree.h>
class HybridJunctionTree {
  HybridJunctionTree(const gtsam::HybridEliminationTree& eliminationTree);
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/DCSAM.h>
class DCSAM {
  DCSAM();
  DCSAM(const gtsam::ISAM2Params& isam_params);

  void update();
  void update(const gtsam::HybridNonlinearFactorGraph& graph,
              const gtsam::HybridValues& initialGuess = gtsam::HybridValues());
  void update(const gtsam::HybridNonlinearFactorGraph& graph,
              const gtsam::DiscreteValues& initialGuessDiscrete);

  gtsam::HybridValues calculateEstimate() const;

  const gtsam::DiscreteFactorGraph& getDiscreteFactorGraph() const;
  const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const;

  const gtsam::VectorValues& getDelta() const;
  double error(const gtsam::VectorValues& x) const;

};

}  // namespace gtsam
