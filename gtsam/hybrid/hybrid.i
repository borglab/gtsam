//*************************************************************************
// hybrid
//*************************************************************************

namespace gtsam {

#include <gtsam/hybrid/HybridValues.h>
class HybridValues {
  gtsam::VectorValues continuous() const;
  gtsam::DiscreteValues discrete() const;

  HybridValues();
  HybridValues(const gtsam::VectorValues &cv, const gtsam::DiscreteValues &dv);
  void print(string s = "HybridValues",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridValues& other, double tol) const;
  
  void insert(gtsam::Key j, int value);
  void insert(gtsam::Key j, const gtsam::Vector& value);
  
  void insert_or_assign(gtsam::Key j, const gtsam::Vector& value);
  void insert_or_assign(gtsam::Key j, size_t value);

  void insert(const gtsam::VectorValues& values);
  void insert(const gtsam::DiscreteValues& values);
  void insert(const gtsam::HybridValues& values);
  
  void update(const gtsam::VectorValues& values);
  void update(const gtsam::DiscreteValues& values);
  void update(const gtsam::HybridValues& values);
  
  size_t& atDiscrete(gtsam::Key j);
  gtsam::Vector& at(gtsam::Key j);
};

#include <gtsam/hybrid/HybridFactor.h>
virtual class HybridFactor : gtsam::Factor {
  void print(string s = "HybridFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridFactor& other, double tol = 1e-9) const;

  // Standard interface:
  double error(const gtsam::HybridValues &values) const;
  bool isDiscrete() const;
  bool isContinuous() const;
  bool isHybrid() const;
  size_t nrContinuous() const;
  gtsam::DiscreteKeys discreteKeys() const;
  gtsam::KeyVector continuousKeys() const;
};

#include <gtsam/hybrid/HybridConditional.h>
virtual class HybridConditional {
  void print(string s = "Hybrid Conditional\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridConditional& other, double tol = 1e-9) const;
  size_t nrFrontals() const;
  size_t nrParents() const;

  // Standard interface:
  double logNormalizationConstant() const;
  double logProbability(const gtsam::HybridValues& values) const;
  double evaluate(const gtsam::HybridValues& values) const;
  double operator()(const gtsam::HybridValues& values) const;
  gtsam::HybridGaussianConditional* asMixture() const;
  gtsam::GaussianConditional* asGaussian() const;
  gtsam::DiscreteConditional* asDiscrete() const;
  gtsam::Factor* inner();
  double error(const gtsam::HybridValues& values) const;
};

#include <gtsam/hybrid/HybridGaussianFactor.h>
class HybridGaussianFactor : gtsam::HybridFactor {
  HybridGaussianFactor(
      const gtsam::KeyVector& continuousKeys,
      const gtsam::DiscreteKeys& discreteKeys,
      const std::vector<gtsam::GaussianFactor::shared_ptr>& factorsList);

  void print(string s = "HybridGaussianFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/HybridGaussianConditional.h>
class HybridGaussianConditional : gtsam::HybridFactor {
  HybridGaussianConditional(const gtsam::KeyVector& continuousFrontals,
                  const gtsam::KeyVector& continuousParents,
                  const gtsam::DiscreteKeys& discreteParents,
                  const std::vector<gtsam::GaussianConditional::shared_ptr>&
                      conditionalsList);

  gtsam::HybridGaussianFactor* likelihood(
      const gtsam::VectorValues& frontals) const;
  double logProbability(const gtsam::HybridValues& values) const;
  double evaluate(const gtsam::HybridValues& values) const;

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

  bool empty() const;
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::HybridConditional* at(size_t i) const;
  
  // Standard interface:
  double logProbability(const gtsam::HybridValues& values) const;
  double evaluate(const gtsam::HybridValues& values) const;
  double error(const gtsam::HybridValues& values) const;

  gtsam::HybridGaussianFactorGraph toFactorGraph(
      const gtsam::VectorValues& measurements) const;

  gtsam::HybridValues optimize() const;
  gtsam::HybridValues sample(const gtsam::HybridValues &given) const;
  gtsam::HybridValues sample() const;

  void print(string s = "HybridBayesNet\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridBayesNet& other, double tol = 1e-9) const;

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
  bool equals(const gtsam::HybridGaussianFactorGraph& fg, double tol = 1e-9) const;

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

#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
class HybridNonlinearFactorGraph {
  HybridNonlinearFactorGraph();
  HybridNonlinearFactorGraph(const gtsam::HybridNonlinearFactorGraph& graph);
  void push_back(gtsam::HybridFactor* factor);
  void push_back(gtsam::NonlinearFactor* factor);
  void push_back(gtsam::DiscreteFactor* factor);
  gtsam::HybridGaussianFactorGraph linearize(const gtsam::Values& continuousValues) const;

  bool empty() const;
  void remove(size_t i);
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::HybridFactor* at(size_t i) const;

  void print(string s = "HybridNonlinearFactorGraph\n",
       const gtsam::KeyFormatter& keyFormatter =
       gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/HybridNonlinearFactor.h>
class HybridNonlinearFactor : gtsam::HybridFactor {
  HybridNonlinearFactor(
      const gtsam::KeyVector& keys, const gtsam::DiscreteKeys& discreteKeys,
      const gtsam::DecisionTree<gtsam::Key, gtsam::NonlinearFactor*>& factors,
      bool normalized = false);

  template <FACTOR = {gtsam::NonlinearFactor}>
  HybridNonlinearFactor(const gtsam::KeyVector& keys, const gtsam::DiscreteKeys& discreteKeys,
                const std::vector<FACTOR*>& factors,
                bool normalized = false);

  double error(const gtsam::Values& continuousValues,
               const gtsam::DiscreteValues& discreteValues) const;

  double nonlinearFactorLogNormalizingConstant(const gtsam::NonlinearFactor* factor,
                                               const gtsam::Values& values) const;

  HybridGaussianFactor* linearize(
      const gtsam::Values& continuousValues) const;

  void print(string s = "HybridNonlinearFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

}  // namespace gtsam
