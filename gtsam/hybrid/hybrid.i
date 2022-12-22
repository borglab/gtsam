//*************************************************************************
// hybrid
//*************************************************************************

namespace gtsam {

#include <gtsam/hybrid/HybridValues.h>
class HybridValues {
  gtsam::DiscreteValues discrete() const;
  gtsam::VectorValues continuous() const;
  HybridValues();
  HybridValues(const gtsam::DiscreteValues &dv, const gtsam::VectorValues &cv);
  void print(string s = "HybridValues",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridValues& other, double tol) const;
  void insert(gtsam::Key j, int value);
  void insert(gtsam::Key j, const gtsam::Vector& value);
  size_t& atDiscrete(gtsam::Key j);
  gtsam::Vector& at(gtsam::Key j);
};

#include <gtsam/hybrid/HybridFactor.h>
virtual class HybridFactor {
  void print(string s = "HybridFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridFactor& other, double tol = 1e-9) const;
  bool empty() const;
  size_t size() const;
  gtsam::KeyVector keys() const;
};

#include <gtsam/hybrid/HybridConditional.h>
virtual class HybridConditional {
  void print(string s = "Hybrid Conditional\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridConditional& other, double tol = 1e-9) const;
  size_t nrFrontals() const;
  size_t nrParents() const;
  gtsam::Factor* inner();
};

#include <gtsam/hybrid/HybridDiscreteFactor.h>
virtual class HybridDiscreteFactor {
  HybridDiscreteFactor(gtsam::DecisionTreeFactor dtf);
  void print(string s = "HybridDiscreteFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::HybridDiscreteFactor& other, double tol = 1e-9) const;
  gtsam::Factor* inner();
};

#include <gtsam/hybrid/GaussianMixtureFactor.h>
class GaussianMixtureFactor : gtsam::HybridFactor {
  static GaussianMixtureFactor FromFactors(
      const gtsam::KeyVector& continuousKeys,
      const gtsam::DiscreteKeys& discreteKeys,
      const std::vector<gtsam::GaussianFactor::shared_ptr>& factorsList);

  void print(string s = "GaussianMixtureFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/hybrid/GaussianMixture.h>
class GaussianMixture : gtsam::HybridFactor {
  static GaussianMixture FromConditionals(
      const gtsam::KeyVector& continuousFrontals,
      const gtsam::KeyVector& continuousParents,
      const gtsam::DiscreteKeys& discreteParents,
      const std::vector<gtsam::GaussianConditional::shared_ptr>&
          conditionalsList);

  void print(string s = "GaussianMixture\n",
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

#include <gtsam/hybrid/HybridBayesTree.h>
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

class HybridBayesNet {
  HybridBayesNet();
  void add(const gtsam::HybridConditional& s);
  bool empty() const;
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::HybridConditional* at(size_t i) const;
  gtsam::HybridValues optimize() const;
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
  void push_back(const gtsam::GaussianMixtureFactor* gmm);

  void add(gtsam::DecisionTreeFactor* factor);
  void add(gtsam::JacobianFactor* factor);

  bool empty() const;
  void remove(size_t i);
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::HybridFactor* at(size_t i) const;

  void print(string s = "") const;
  bool equals(const gtsam::HybridGaussianFactorGraph& fg, double tol = 1e-9) const;

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
  void push_back(gtsam::HybridDiscreteFactor* factor);
  void add(gtsam::NonlinearFactor* factor);
  void add(gtsam::DiscreteFactor* factor);
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

#include <gtsam/hybrid/MixtureFactor.h>
class MixtureFactor : gtsam::HybridFactor {
  MixtureFactor(const gtsam::KeyVector& keys, const gtsam::DiscreteKeys& discreteKeys,
                const gtsam::DecisionTree<gtsam::Key, gtsam::NonlinearFactor*>& factors, bool normalized = false);

  template <FACTOR = {gtsam::NonlinearFactor}>
  MixtureFactor(const gtsam::KeyVector& keys, const gtsam::DiscreteKeys& discreteKeys,
                const std::vector<FACTOR*>& factors,
                bool normalized = false);

  double error(const gtsam::Values& continuousValues,
               const gtsam::DiscreteValues& discreteValues) const;

  double nonlinearFactorLogNormalizingConstant(const gtsam::NonlinearFactor* factor,
                                               const gtsam::Values& values) const;

  GaussianMixtureFactor* linearize(
      const gtsam::Values& continuousValues) const;

  void print(string s = "MixtureFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

}  // namespace gtsam
