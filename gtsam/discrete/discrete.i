//*************************************************************************
// discrete
//*************************************************************************

namespace gtsam {


#include<gtsam/discrete/DiscreteKey.h>
class DiscreteKey {};

class DiscreteKeys {
  DiscreteKeys();
  size_t size() const;
  bool empty() const;
  gtsam::DiscreteKey at(size_t n) const;
  void push_back(const gtsam::DiscreteKey& point_pair);
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

// DiscreteValues is added in specializations/discrete.h as a std::map
std::vector<gtsam::DiscreteValues> cartesianProduct(
    const gtsam::DiscreteKeys& keys);
string markdown(
    const gtsam::DiscreteValues& values,
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
string markdown(const gtsam::DiscreteValues& values,
                const gtsam::KeyFormatter& keyFormatter,
                std::map<gtsam::Key, std::vector<std::string>> names);
string html(
    const gtsam::DiscreteValues& values,
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
string html(const gtsam::DiscreteValues& values,
            const gtsam::KeyFormatter& keyFormatter,
            std::map<gtsam::Key, std::vector<std::string>> names);

#include <gtsam/discrete/DiscreteFactor.h>
virtual class DiscreteFactor : gtsam::Factor {
  void print(string s = "DiscreteFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol = 1e-9) const;
  double operator()(const gtsam::DiscreteValues& values) const;
};

#include <gtsam/discrete/DecisionTreeFactor.h>
virtual class DecisionTreeFactor : gtsam::DiscreteFactor {
  DecisionTreeFactor();

  DecisionTreeFactor(const gtsam::DiscreteKey& key,
                     const std::vector<double>& spec);
  DecisionTreeFactor(const gtsam::DiscreteKey& key, string table);

  DecisionTreeFactor(const gtsam::DiscreteKeys& keys,
                     const std::vector<double>& table);
  DecisionTreeFactor(const gtsam::DiscreteKeys& keys, string table);

  DecisionTreeFactor(const std::vector<gtsam::DiscreteKey>& keys,
                     const std::vector<double>& table);
  DecisionTreeFactor(const std::vector<gtsam::DiscreteKey>& keys, string table);

  DecisionTreeFactor(const gtsam::DiscreteConditional& c);
  
  void print(string s = "DecisionTreeFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DecisionTreeFactor& other, double tol = 1e-9) const;

  size_t cardinality(gtsam::Key j) const;
  
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor operator*(const gtsam::DecisionTreeFactor& f) const;
  size_t cardinality(gtsam::Key j) const;
  gtsam::DecisionTreeFactor operator/(const gtsam::DecisionTreeFactor& f) const;
  gtsam::DecisionTreeFactor* sum(size_t nrFrontals) const;
  gtsam::DecisionTreeFactor* sum(const gtsam::Ordering& keys) const;
  gtsam::DecisionTreeFactor* max(size_t nrFrontals) const;
  gtsam::DecisionTreeFactor* max(const gtsam::Ordering& keys) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      bool showZero = true) const;
  std::vector<std::pair<gtsam::DiscreteValues, double>> enumerate() const;
  string markdown(const gtsam::KeyFormatter& keyFormatter =
                      gtsam::DefaultKeyFormatter) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter,
                  std::map<gtsam::Key, std::vector<std::string>> names) const;
  string html(const gtsam::KeyFormatter& keyFormatter =
                  gtsam::DefaultKeyFormatter) const;
  string html(const gtsam::KeyFormatter& keyFormatter,
              std::map<gtsam::Key, std::vector<std::string>> names) const;
};

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/hybrid/HybridValues.h>
virtual class DiscreteConditional : gtsam::DecisionTreeFactor {
  DiscreteConditional();
  DiscreteConditional(size_t nFrontals, const gtsam::DecisionTreeFactor& f);
  DiscreteConditional(const gtsam::DiscreteKey& key, string spec);
  DiscreteConditional(const gtsam::DiscreteKey& key,
                      const gtsam::DiscreteKeys& parents, string spec);
  DiscreteConditional(const gtsam::DiscreteKey& key,
                      const std::vector<gtsam::DiscreteKey>& parents, string spec);
  DiscreteConditional(const gtsam::DecisionTreeFactor& joint,
                      const gtsam::DecisionTreeFactor& marginal);
  DiscreteConditional(const gtsam::DecisionTreeFactor& joint,
                      const gtsam::DecisionTreeFactor& marginal,
                      const gtsam::Ordering& orderedKeys);
  DiscreteConditional(const gtsam::DiscreteKey& key,
                      const gtsam::DiscreteKeys& parents,
                      const std::vector<double>& table);

  // Standard interface
  double negLogConstant() const;
  double logProbability(const gtsam::DiscreteValues& values) const;
  double evaluate(const gtsam::DiscreteValues& values) const;
  double error(const gtsam::DiscreteValues& values) const;
  gtsam::DiscreteConditional operator*(
      const gtsam::DiscreteConditional& other) const;
  gtsam::DiscreteConditional marginal(gtsam::Key key) const;
  void print(string s = "Discrete Conditional\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteConditional& other, double tol = 1e-9) const;
  gtsam::Key firstFrontalKey() const;
  size_t nrFrontals() const;
  size_t nrParents() const;
  void printSignature(
      string s = "Discrete Conditional: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  gtsam::DecisionTreeFactor* choose(const gtsam::DiscreteValues& given) const;
  gtsam::DecisionTreeFactor* likelihood(
      const gtsam::DiscreteValues& frontalValues) const;
  gtsam::DecisionTreeFactor* likelihood(size_t value) const;
  size_t sample(const gtsam::DiscreteValues& parentsValues) const;
  size_t sample(size_t value) const;
  size_t sample() const;
  void sampleInPlace(gtsam::DiscreteValues @parentsValues) const;
  size_t argmax(const gtsam::DiscreteValues& parents) const;

  // Markdown and HTML
  string markdown(const gtsam::KeyFormatter& keyFormatter =
                      gtsam::DefaultKeyFormatter) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter,
                  std::map<gtsam::Key, std::vector<std::string>> names) const;
  string html(const gtsam::KeyFormatter& keyFormatter =
                  gtsam::DefaultKeyFormatter) const;
  string html(const gtsam::KeyFormatter& keyFormatter,
              std::map<gtsam::Key, std::vector<std::string>> names) const;

  // Expose HybridValues versions
  double logProbability(const gtsam::HybridValues& x) const;
  double evaluate(const gtsam::HybridValues& x) const;
  double error(const gtsam::HybridValues& x) const;
};

#include <gtsam/discrete/DiscreteDistribution.h>
virtual class DiscreteDistribution : gtsam::DiscreteConditional {
  DiscreteDistribution();
  DiscreteDistribution(const gtsam::DecisionTreeFactor& f);
  DiscreteDistribution(const gtsam::DiscreteKey& key, string spec);
  DiscreteDistribution(const gtsam::DiscreteKey& key, std::vector<double> spec);
  void print(string s = "Discrete Prior\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  double operator()(size_t value) const;
  std::vector<double> pmf() const;
};

#include <gtsam/discrete/DiscreteBayesNet.h>
class DiscreteBayesNet {
  DiscreteBayesNet();
  void add(const gtsam::DiscreteConditional& s);
  void add(const gtsam::DiscreteKey& key, string spec);
  void add(const gtsam::DiscreteKey& key, const gtsam::DiscreteKeys& parents,
           string spec);
  void add(const gtsam::DiscreteKey& key,
           const std::vector<gtsam::DiscreteKey>& parents, string spec);
  bool empty() const;
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::DiscreteConditional* at(size_t i) const;
  void print(string s = "DiscreteBayesNet\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteBayesNet& other, double tol = 1e-9) const;

  // Standard interface.
  double logProbability(const gtsam::DiscreteValues& values) const;
  double evaluate(const gtsam::DiscreteValues& values) const;
  double operator()(const gtsam::DiscreteValues& values) const;

  gtsam::DiscreteValues sample() const;
  gtsam::DiscreteValues sample(gtsam::DiscreteValues given) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      string s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter =
                      gtsam::DefaultKeyFormatter) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter,
                  std::map<gtsam::Key, std::vector<std::string>> names) const;
  string html(const gtsam::KeyFormatter& keyFormatter =
                  gtsam::DefaultKeyFormatter) const;
  string html(const gtsam::KeyFormatter& keyFormatter,
              std::map<gtsam::Key, std::vector<std::string>> names) const;
};

#include <gtsam/discrete/DiscreteBayesTree.h>
class DiscreteBayesTreeClique {
  DiscreteBayesTreeClique();
  DiscreteBayesTreeClique(const gtsam::DiscreteConditional* conditional);
  const gtsam::DiscreteConditional* conditional() const;
  bool isRoot() const;
  size_t nrChildren() const;
  const gtsam::DiscreteBayesTreeClique* operator[](size_t i) const;
  void print(string s = "DiscreteBayesTreeClique",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void printSignature(
      const string& s = "Clique: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  double evaluate(const gtsam::DiscreteValues& values) const;
  double operator()(const gtsam::DiscreteValues& values) const;
};

class DiscreteBayesTree {
  DiscreteBayesTree();
  void print(string s = "DiscreteBayesTree\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteBayesTree& other, double tol = 1e-9) const;

  size_t size() const;
  bool empty() const;
  const DiscreteBayesTreeClique* operator[](size_t j) const;

  double evaluate(const gtsam::DiscreteValues& values) const;
  double operator()(const gtsam::DiscreteValues& values) const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void saveGraph(string s,
                const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  double operator()(const gtsam::DiscreteValues& values) const;

  string markdown(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter,
                  std::map<gtsam::Key, std::vector<std::string>> names) const;
  string html(const gtsam::KeyFormatter& keyFormatter =
                  gtsam::DefaultKeyFormatter) const;
  string html(const gtsam::KeyFormatter& keyFormatter,
              std::map<gtsam::Key, std::vector<std::string>> names) const;
};

#include <gtsam/discrete/DiscreteLookupDAG.h>

class DiscreteLookupTable : gtsam::DiscreteConditional{
  DiscreteLookupTable(size_t nFrontals, const gtsam::DiscreteKeys& keys,
                      const gtsam::DecisionTreeFactor::ADT& potentials);
  void print(string s = "Discrete Lookup Table: ",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  size_t argmax(const gtsam::DiscreteValues& parentsValues) const;
};

class DiscreteLookupDAG {
  DiscreteLookupDAG();
  void push_back(const gtsam::DiscreteLookupTable* table);
  bool empty() const;
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::DiscreteLookupTable* at(size_t i) const;
  void print(string s = "DiscreteLookupDAG\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  gtsam::DiscreteValues argmax() const;
  gtsam::DiscreteValues argmax(gtsam::DiscreteValues given) const;
};

#include <gtsam/discrete/DiscreteFactorGraph.h>
pair<gtsam::DiscreteConditional*, gtsam::DecisionTreeFactor*>
EliminateDiscrete(const gtsam::DiscreteFactorGraph& factors,
                  const gtsam::Ordering& frontalKeys);

pair<gtsam::DiscreteConditional*, gtsam::DecisionTreeFactor*>
EliminateForMPE(const gtsam::DiscreteFactorGraph& factors,
                const gtsam::Ordering& frontalKeys);

class DiscreteFactorGraph {
  DiscreteFactorGraph();
  DiscreteFactorGraph(const gtsam::DiscreteBayesNet& bayesNet);

  // Building the graph
  void push_back(const gtsam::DiscreteFactor* factor);
  void push_back(const gtsam::DiscreteConditional* conditional);
  void push_back(const gtsam::DiscreteFactorGraph& graph);
  void push_back(const gtsam::DiscreteBayesNet& bayesNet);
  void push_back(const gtsam::DiscreteBayesTree& bayesTree);
  void add(const gtsam::DiscreteKey& j, string spec);
  void add(const gtsam::DiscreteKey& j, const std::vector<double>& spec);
  void add(const gtsam::DiscreteKeys& keys, string spec);
  void add(const std::vector<gtsam::DiscreteKey>& keys, string spec);
  void add(const std::vector<gtsam::DiscreteKey>& keys, const std::vector<double>& spec);

  bool empty() const;
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::DiscreteFactor* at(size_t i) const;

  void print(string s = "") const;
  bool equals(const gtsam::DiscreteFactorGraph& fg, double tol = 1e-9) const;

  gtsam::DecisionTreeFactor product() const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DiscreteValues optimize() const;

  gtsam::DiscreteBayesNet sumProduct(
      gtsam::Ordering::OrderingType type = gtsam::Ordering::COLAMD);
  gtsam::DiscreteBayesNet sumProduct(const gtsam::Ordering& ordering);

  gtsam::DiscreteLookupDAG maxProduct(
      gtsam::Ordering::OrderingType type = gtsam::Ordering::COLAMD);
  gtsam::DiscreteLookupDAG maxProduct(const gtsam::Ordering& ordering);

  gtsam::DiscreteBayesNet* eliminateSequential(
      gtsam::Ordering::OrderingType type = gtsam::Ordering::COLAMD);
  gtsam::DiscreteBayesNet* eliminateSequential(
      gtsam::Ordering::OrderingType type,
      const gtsam::DiscreteFactorGraph::Eliminate& function);
  gtsam::DiscreteBayesNet* eliminateSequential(const gtsam::Ordering& ordering);
  gtsam::DiscreteBayesNet* eliminateSequential(
      const gtsam::Ordering& ordering,
      const gtsam::DiscreteFactorGraph::Eliminate& function);
  pair<gtsam::DiscreteBayesNet*, gtsam::DiscreteFactorGraph*>
  eliminatePartialSequential(const gtsam::Ordering& ordering);
  pair<gtsam::DiscreteBayesNet*, gtsam::DiscreteFactorGraph*>
  eliminatePartialSequential(
      const gtsam::Ordering& ordering,
      const gtsam::DiscreteFactorGraph::Eliminate& function);

  gtsam::DiscreteBayesTree* eliminateMultifrontal(
      gtsam::Ordering::OrderingType type = gtsam::Ordering::COLAMD);
  gtsam::DiscreteBayesTree* eliminateMultifrontal(
      gtsam::Ordering::OrderingType type,
      const gtsam::DiscreteFactorGraph::Eliminate& function);
  gtsam::DiscreteBayesTree* eliminateMultifrontal(
      const gtsam::Ordering& ordering);
  gtsam::DiscreteBayesTree* eliminateMultifrontal(
      const gtsam::Ordering& ordering,
      const gtsam::DiscreteFactorGraph::Eliminate& function);
  pair<gtsam::DiscreteBayesTree*, gtsam::DiscreteFactorGraph*>
  eliminatePartialMultifrontal(const gtsam::Ordering& ordering);
  pair<gtsam::DiscreteBayesTree*, gtsam::DiscreteFactorGraph*>
  eliminatePartialMultifrontal(
      const gtsam::Ordering& ordering,
      const gtsam::DiscreteFactorGraph::Eliminate& function);

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      string s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;

  string markdown(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter,
                  std::map<gtsam::Key, std::vector<std::string>> names) const;
  string html(const gtsam::KeyFormatter& keyFormatter =
                  gtsam::DefaultKeyFormatter) const;
  string html(const gtsam::KeyFormatter& keyFormatter,
              std::map<gtsam::Key, std::vector<std::string>> names) const;
};

#include <gtsam/discrete/DiscreteEliminationTree.h>

class DiscreteEliminationTree {
  DiscreteEliminationTree(const gtsam::DiscreteFactorGraph& factorGraph,
                          const gtsam::VariableIndex& structure,
                          const gtsam::Ordering& order);

  DiscreteEliminationTree(const gtsam::DiscreteFactorGraph& factorGraph,
                          const gtsam::Ordering& order);

  void print(
      string name = "EliminationTree: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteEliminationTree& other,
              double tol = 1e-9) const;
};

#include <gtsam/discrete/DiscreteJunctionTree.h>

class DiscreteCluster {
  gtsam::Ordering orderedFrontalKeys;
  gtsam::DiscreteFactorGraph factors;
  const gtsam::DiscreteCluster& operator[](size_t i) const;
  size_t nrChildren() const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
};

class DiscreteJunctionTree {
  DiscreteJunctionTree(const gtsam::DiscreteEliminationTree& eliminationTree);
  void print(
      string name = "JunctionTree: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  size_t nrRoots() const;
  const gtsam::DiscreteCluster& operator[](size_t i) const;
};

}  // namespace gtsam
