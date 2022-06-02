//*************************************************************************
// hybrid
//*************************************************************************

namespace gtsam {

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
  Factor* inner();
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

}  // namespace gtsam
