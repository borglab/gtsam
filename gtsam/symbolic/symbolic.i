//*************************************************************************
// Symbolic
//*************************************************************************
namespace gtsam {

#include <gtsam/symbolic/SymbolicFactor.h>
virtual class SymbolicFactor : gtsam::Factor {
  // Standard Constructors and Named Constructors
  SymbolicFactor(const gtsam::SymbolicFactor& f);
  SymbolicFactor();
  SymbolicFactor(gtsam::Key j);
  SymbolicFactor(gtsam::Key j1, gtsam::Key j2);
  SymbolicFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3);
  SymbolicFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4);
  SymbolicFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4, gtsam::Key j5);
  SymbolicFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4, gtsam::Key j5,
                 gtsam::Key j6);
  static gtsam::SymbolicFactor FromKeys(const gtsam::KeyVector& keys);

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
  bool equals(const gtsam::SymbolicFactorGraph& fg, double tol) const;
  size_t size() const;
  bool exists(size_t idx) const;

  // Standard interface
  gtsam::KeySet keys() const;
  void push_back(const gtsam::SymbolicFactorGraph& graph);
  void push_back(const gtsam::SymbolicBayesNet& bayesNet);
  void push_back(const gtsam::SymbolicBayesTree& bayesTree);

  // Advanced Interface
  void push_factor(gtsam::Key key);
  void push_factor(gtsam::Key key1, gtsam::Key key2);
  void push_factor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3);
  void push_factor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4);

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
  SymbolicConditional(gtsam::Key key);
  SymbolicConditional(gtsam::Key key, gtsam::Key parent);
  SymbolicConditional(gtsam::Key key, gtsam::Key parent1, gtsam::Key parent2);
  SymbolicConditional(gtsam::Key key, gtsam::Key parent1, gtsam::Key parent2,
                      gtsam::Key parent3);
  static gtsam::SymbolicConditional FromKeys(const gtsam::KeyVector& keys,
                                             size_t nrFrontals);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::SymbolicConditional& c, double tol) const;

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
  bool equals(const gtsam::SymbolicBayesNet& bn, double tol) const;

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

#include <gtsam/symbolic/SymbolicEliminationTree.h>

class SymbolicEliminationTree {
  SymbolicEliminationTree(const gtsam::SymbolicFactorGraph& factorGraph,
                          const gtsam::VariableIndex& structure,
                          const gtsam::Ordering& order);

  SymbolicEliminationTree(const gtsam::SymbolicFactorGraph& factorGraph,
                          const gtsam::Ordering& order);

  void print(
      string name = "EliminationTree: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::SymbolicEliminationTree& other,
              double tol = 1e-9) const;
};

#include <gtsam/symbolic/SymbolicJunctionTree.h>

class SymbolicCluster {
  gtsam::Ordering orderedFrontalKeys;
  gtsam::SymbolicFactorGraph factors;
  const gtsam::SymbolicCluster& operator[](size_t i) const;
  size_t nrChildren() const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
};

class SymbolicJunctionTree {
  SymbolicJunctionTree(const gtsam::SymbolicEliminationTree& eliminationTree);
  void print(
      string name = "JunctionTree: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  size_t nrRoots() const;
  const gtsam::SymbolicCluster& operator[](size_t i) const;
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
  size_t nrChildren() const;
  gtsam::SymbolicBayesTreeClique* operator[](size_t j) const;
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
  const SymbolicBayesTree::Roots& roots() const;
  const gtsam::SymbolicBayesTreeClique* operator[](size_t j) const;

  void saveGraph(string s,
                const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void clear();
  void deleteCachedShortcuts();
  size_t numCachedSeparatorMarginals() const;

  gtsam::SymbolicConditional* marginalFactor(gtsam::Key key) const;
  gtsam::SymbolicFactorGraph* joint(gtsam::Key key1, gtsam::Key key2) const;
  gtsam::SymbolicBayesNet* jointBayesNet(gtsam::Key key1, gtsam::Key key2) const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

}  // namespace gtsam
