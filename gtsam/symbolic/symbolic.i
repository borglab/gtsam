//*************************************************************************
// Symbolic
//*************************************************************************
namespace gtsam {

#include <gtsam/symbolic/SymbolicFactor.h>
virtual class SymbolicFactor : gtsam::Factor {
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
  void print(string s = "SymbolicFactor",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::SymbolicFactor& other, double tol) const;
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

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      string s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
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
  gtsam::Key firstFrontalKey() const;
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

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      string s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
};

#include <gtsam/symbolic/SymbolicBayesTree.h>

class SymbolicBayesTreeClique {
  SymbolicBayesTreeClique();
  SymbolicBayesTreeClique(const gtsam::SymbolicConditional* conditional);
  bool equals(const gtsam::SymbolicBayesTreeClique& other, double tol) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter);
  const gtsam::SymbolicConditional* conditional() const;
  bool isRoot() const;
  gtsam::SymbolicBayesTreeClique* parent() const;
  size_t treeSize() const;
  size_t numCachedSeparatorMarginals() const;
  void deleteCachedShortcuts();
};


class SymbolicBayesTree {
  // Constructors
  SymbolicBayesTree();
  SymbolicBayesTree(const gtsam::SymbolicBayesTree& other);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter);
  bool equals(const gtsam::SymbolicBayesTree& other, double tol) const;

  // Standard Interface
  bool empty() const;
  size_t size() const;

  const gtsam::SymbolicBayesTreeClique* operator[](size_t j) const;

  void saveGraph(string s,
                const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void clear();
  void deleteCachedShortcuts();
  size_t numCachedSeparatorMarginals() const;

  gtsam::SymbolicConditional* marginalFactor(size_t key) const;
  gtsam::SymbolicFactorGraph* joint(size_t key1, size_t key2) const;
  gtsam::SymbolicBayesNet* jointBayesNet(size_t key1, size_t key2) const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

}  // namespace gtsam
