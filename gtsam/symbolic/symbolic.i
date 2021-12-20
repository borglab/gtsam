//*************************************************************************
// Symbolic
//*************************************************************************
namespace gtsam {

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// ###################

#include <gtsam/symbolic/SymbolicFactor.h>
virtual class SymbolicFactor {
  // Standard Constructors and Named Constructors
  SymbolicFactor(const gtsam::SymbolicFactor& f);
  SymbolicFactor();
  SymbolicFactor(size_t j);
  SymbolicFactor(size_t j1, size_t j2);
  SymbolicFactor(size_t j1, size_t j2, size_t j3);
  SymbolicFactor(size_t j1, size_t j2, size_t j3, size_t j4);
  SymbolicFactor(size_t j1, size_t j2, size_t j3, size_t j4, size_t j5);
  SymbolicFactor(size_t j1, size_t j2, size_t j3, size_t j4, size_t j5,
                 size_t j6);
  static gtsam::SymbolicFactor FromKeys(const gtsam::KeyVector& js);

  // From Factor
  size_t size() const;
  void print(string s = "SymbolicFactor",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::SymbolicFactor& other, double tol) const;
  gtsam::KeyVector keys();
};

#include <gtsam/symbolic/SymbolicFactorGraph.h>
virtual class SymbolicFactorGraph {
  SymbolicFactorGraph();
  SymbolicFactorGraph(const gtsam::SymbolicBayesNet& bayesNet);
  SymbolicFactorGraph(const gtsam::SymbolicBayesTree& bayesTree);

  // From FactorGraph
  void push_back(gtsam::SymbolicFactor* factor);
  void print(string s = "SymbolicFactorGraph",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::SymbolicFactorGraph& rhs, double tol) const;
  size_t size() const;
  bool exists(size_t idx) const;

  // Standard interface
  gtsam::KeySet keys() const;
  void push_back(const gtsam::SymbolicFactorGraph& graph);
  void push_back(const gtsam::SymbolicBayesNet& bayesNet);
  void push_back(const gtsam::SymbolicBayesTree& bayesTree);

  // Advanced Interface
  void push_factor(size_t key);
  void push_factor(size_t key1, size_t key2);
  void push_factor(size_t key1, size_t key2, size_t key3);
  void push_factor(size_t key1, size_t key2, size_t key3, size_t key4);

  gtsam::SymbolicBayesNet* eliminateSequential();
  gtsam::SymbolicBayesNet* eliminateSequential(const gtsam::Ordering& ordering);
  gtsam::SymbolicBayesTree* eliminateMultifrontal();
  gtsam::SymbolicBayesTree* eliminateMultifrontal(
      const gtsam::Ordering& ordering);
  pair<gtsam::SymbolicBayesNet*, gtsam::SymbolicFactorGraph*>
  eliminatePartialSequential(const gtsam::Ordering& ordering);
  pair<gtsam::SymbolicBayesNet*, gtsam::SymbolicFactorGraph*>
  eliminatePartialSequential(const gtsam::KeyVector& keys);
  pair<gtsam::SymbolicBayesTree*, gtsam::SymbolicFactorGraph*>
  eliminatePartialMultifrontal(const gtsam::Ordering& ordering);
  pair<gtsam::SymbolicBayesTree*, gtsam::SymbolicFactorGraph*>
  eliminatePartialMultifrontal(const gtsam::KeyVector& keys);
  gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(
      const gtsam::Ordering& ordering);
  gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(
      const gtsam::KeyVector& key_vector);
  gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(
      const gtsam::Ordering& ordering,
      const gtsam::Ordering& marginalizedVariableOrdering);
  gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(
      const gtsam::KeyVector& key_vector,
      const gtsam::Ordering& marginalizedVariableOrdering);
  gtsam::SymbolicFactorGraph* marginal(const gtsam::KeyVector& key_vector);
};

#include <gtsam/symbolic/SymbolicConditional.h>
virtual class SymbolicConditional : gtsam::SymbolicFactor {
  // Standard Constructors and Named Constructors
  SymbolicConditional();
  SymbolicConditional(const gtsam::SymbolicConditional& other);
  SymbolicConditional(size_t key);
  SymbolicConditional(size_t key, size_t parent);
  SymbolicConditional(size_t key, size_t parent1, size_t parent2);
  SymbolicConditional(size_t key, size_t parent1, size_t parent2,
                      size_t parent3);
  static gtsam::SymbolicConditional FromKeys(const gtsam::KeyVector& keys,
                                             size_t nrFrontals);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::SymbolicConditional& other, double tol) const;

  // Standard interface
  size_t nrFrontals() const;
  size_t nrParents() const;
};

#include <gtsam/symbolic/SymbolicBayesNet.h>
class SymbolicBayesNet {
  SymbolicBayesNet();
  SymbolicBayesNet(const gtsam::SymbolicBayesNet& other);
  // Testable
  void print(string s = "SymbolicBayesNet",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::SymbolicBayesNet& other, double tol) const;

  // Standard interface
  size_t size() const;
  void saveGraph(string s) const;
  gtsam::SymbolicConditional* at(size_t idx) const;
  gtsam::SymbolicConditional* front() const;
  gtsam::SymbolicConditional* back() const;
  void push_back(gtsam::SymbolicConditional* conditional);
  void push_back(const gtsam::SymbolicBayesNet& bayesNet);
};

#include <gtsam/symbolic/SymbolicBayesTree.h>
class SymbolicBayesTree {
  // Constructors
  SymbolicBayesTree();
  SymbolicBayesTree(const gtsam::SymbolicBayesTree& other);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter);
  bool equals(const gtsam::SymbolicBayesTree& other, double tol) const;

  // Standard Interface
  // size_t findParentClique(const gtsam::IndexVector& parents) const;
  size_t size();
  void saveGraph(string s) const;
  void clear();
  void deleteCachedShortcuts();
  size_t numCachedSeparatorMarginals() const;

  gtsam::SymbolicConditional* marginalFactor(size_t key) const;
  gtsam::SymbolicFactorGraph* joint(size_t key1, size_t key2) const;
  gtsam::SymbolicBayesNet* jointBayesNet(size_t key1, size_t key2) const;
};

class SymbolicBayesTreeClique {
  SymbolicBayesTreeClique();
  // SymbolicBayesTreeClique(gtsam::sharedConditional* conditional);

  bool equals(const gtsam::SymbolicBayesTreeClique& other, double tol) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  size_t numCachedSeparatorMarginals() const;
  // gtsam::sharedConditional* conditional() const;
  bool isRoot() const;
  size_t treeSize() const;
  gtsam::SymbolicBayesTreeClique* parent() const;

  //   // TODO: need wrapped versions graphs, BayesNet
  //  BayesNet<ConditionalType> shortcut(derived_ptr root, Eliminate function)
  //  const; FactorGraph<FactorType> marginal(derived_ptr root, Eliminate
  //  function) const; FactorGraph<FactorType> joint(derived_ptr C2, derived_ptr
  //  root, Eliminate function) const;
  //
  void deleteCachedShortcuts();
};

#include <gtsam/inference/VariableIndex.h>
class VariableIndex {
  // Standard Constructors and Named Constructors
  VariableIndex();
  // TODO: Templetize constructor when wrap supports it
  // template<T = {gtsam::FactorGraph}>
  // VariableIndex(const T& factorGraph, size_t nVariables);
  // VariableIndex(const T& factorGraph);
  VariableIndex(const gtsam::SymbolicFactorGraph& sfg);
  VariableIndex(const gtsam::GaussianFactorGraph& gfg);
  VariableIndex(const gtsam::NonlinearFactorGraph& fg);
  VariableIndex(const gtsam::VariableIndex& other);

  // Testable
  bool equals(const gtsam::VariableIndex& other, double tol) const;
  void print(string s = "VariableIndex: ",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;

  // Standard interface
  size_t size() const;
  size_t nFactors() const;
  size_t nEntries() const;
};

}  // namespace gtsam
