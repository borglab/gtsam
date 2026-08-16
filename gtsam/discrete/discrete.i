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
  const gtsam::DiscreteKey& at(size_t n) const;
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
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
    const std::map<gtsam::Key, std::vector<std::string>>& names =
        std::map<gtsam::Key, std::vector<std::string>>());
string html(
    const gtsam::DiscreteValues& values,
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
    const std::map<gtsam::Key, std::vector<std::string>>& names =
        std::map<gtsam::Key, std::vector<std::string>>());

#include <gtsam/discrete/DiscreteFactor.h>
virtual class DiscreteFactor : gtsam::Factor {
  bool equals(const gtsam::DiscreteFactor& lf, double tol = 1e-9) const;
  double evaluate(const gtsam::Assignment<gtsam::Key>& values) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::AlgebraicDecisionTree<gtsam::Key> errorTree() const;
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

  size_t cardinality(gtsam::Key j) const;

  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor operator*(const gtsam::DecisionTreeFactor& f) const;
  size_t cardinality(gtsam::Key j) const;
  gtsam::DecisionTreeFactor operator/(const gtsam::DecisionTreeFactor& f) const;
  gtsam::DiscreteFactor* sum(size_t nrFrontals) const;
  gtsam::DiscreteFactor* sum(const gtsam::Ordering& keys) const;
  gtsam::DiscreteFactor* max(size_t nrFrontals) const;
  gtsam::DiscreteFactor* max(const gtsam::Ordering& keys) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      bool showZero = true) const;
  std::vector<std::pair<gtsam::DiscreteValues, double>> enumerate() const;
  string markdown(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
  string html(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
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
  // Python does not merge this overload with the inherited evaluate overload.
  double evaluate(const gtsam::Assignment<gtsam::Key>& values) const;
  double error(const gtsam::DiscreteValues& values) const;
  gtsam::DiscreteConditional operator*(
      const gtsam::DiscreteConditional& other) const;
  gtsam::DiscreteConditional marginal(gtsam::Key key) const;
  gtsam::Key firstFrontalKey() const;
  size_t nrFrontals() const;
  size_t nrParents() const;
  void printSignature(
      const string& s = "Discrete Conditional: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  gtsam::DiscreteConditional* choose(
      const gtsam::DiscreteValues& given) const;
  gtsam::DecisionTreeFactor* likelihood(
      const gtsam::DiscreteValues& frontalValues) const;
  gtsam::DecisionTreeFactor* likelihood(size_t value) const;
  size_t sample(const gtsam::DiscreteValues& parentsValues,
                std::mt19937_64 @rng = nullptr) const;
  size_t sample(size_t value,
                std::mt19937_64 @rng = nullptr) const;
  size_t sample(std::mt19937_64 @rng = nullptr) const;
  void sampleInPlace(gtsam::DiscreteValues @parentsValues,
                     std::mt19937_64 @rng = nullptr) const;
  size_t argmax(const gtsam::DiscreteValues& parentsValues) const;

  // Markdown and HTML
  string markdown(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
  string html(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;

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
  double operator()(size_t value) const;
  std::vector<double> pmf() const;
};

#include <gtsam/discrete/TableFactor.h>
virtual class TableFactor : gtsam::DiscreteFactor {
  TableFactor();
  TableFactor(const gtsam::DiscreteKeys& keys,
              const gtsam::TableFactor& potentials);
  TableFactor(const gtsam::DiscreteKeys& keys, std::vector<double>& table);
  TableFactor(const gtsam::DiscreteKeys& keys, string spec);
  TableFactor(const gtsam::DiscreteKeys& keys,
              const gtsam::DecisionTreeFactor& dtf);
  TableFactor(const gtsam::DecisionTreeFactor& dtf);

  double error(const gtsam::DiscreteValues& values) const;
};

#include <gtsam/discrete/TableDistribution.h>
virtual class TableDistribution : gtsam::DiscreteConditional {
  TableDistribution();
  TableDistribution(const gtsam::TableFactor& f);
  TableDistribution(const gtsam::DiscreteKey& key, std::vector<double> spec);
  TableDistribution(const gtsam::DiscreteKeys& keys, std::vector<double> spec);
  TableDistribution(const gtsam::DiscreteKeys& keys, string spec);
  TableDistribution(const gtsam::DiscreteKey& key, string spec);

  gtsam::TableFactor table() const;
  uint64_t nrValues() const;
};

#include <gtsam/discrete/DiscreteBayesNet.h>
class DiscreteBayesNet {
  DiscreteBayesNet();
  void add(const gtsam::DiscreteConditional& s);
  void add(const gtsam::DiscreteKey& key, const string& spec);
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
  bool equals(const gtsam::DiscreteBayesNet& bn, double tol = 1e-9) const;

  // Standard interface.
  double logProbability(const gtsam::DiscreteValues& values) const;
  double evaluate(const gtsam::DiscreteValues& values) const;
  double operator()(const gtsam::DiscreteValues& values) const;

  gtsam::DiscreteValues sample(std::mt19937_64
                               @rng = nullptr) const;
  gtsam::DiscreteValues sample(gtsam::DiscreteValues given,
                               std::mt19937_64
                               @rng = nullptr) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      const string& s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  string markdown(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
  string html(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
};

#include <gtsam/discrete/DiscreteBayesTree.h>
class DiscreteBayesTreeClique {
  DiscreteBayesTreeClique();
  DiscreteBayesTreeClique(const gtsam::DiscreteConditional* conditional);
  const gtsam::DiscreteConditional::shared_ptr& conditional() const;
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
  void insertRoot(
      const std::shared_ptr<gtsam::DiscreteBayesTreeClique>& subtree);
  void addClique(
      const std::shared_ptr<gtsam::DiscreteBayesTreeClique>& clique,
      const std::shared_ptr<gtsam::DiscreteBayesTreeClique>& parent_clique =
          std::shared_ptr<gtsam::DiscreteBayesTreeClique>());

  void print(string s = "DiscreteBayesTree\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteBayesTree& other, double tol = 1e-9) const;

  size_t size() const;
  bool empty() const;
  const DiscreteBayesTreeClique* operator[](gtsam::Key j) const;
  const std::shared_ptr<gtsam::DiscreteBayesTreeClique>& clique(
      gtsam::Key j) const;
  size_t numCachedSeparatorMarginals() const;

  std::shared_ptr<gtsam::DiscreteConditional> marginalFactor(
      gtsam::Key j,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate))
      const;
  std::shared_ptr<gtsam::DiscreteFactorGraph> joint(
      gtsam::Key j1, gtsam::Key j2,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate))
      const;
  std::shared_ptr<gtsam::DiscreteBayesNet> jointBayesNet(
      gtsam::Key j1, gtsam::Key j2,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate))
      const;

  double evaluate(const gtsam::DiscreteValues& values) const;
  double operator()(const gtsam::DiscreteValues& values) const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void saveGraph(const string& s,
                const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;

  string markdown(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
  string html(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
};

#include <gtsam/discrete/DiscreteLookupDAG.h>

class DiscreteLookupTable : gtsam::DiscreteConditional{
  DiscreteLookupTable(size_t nFrontals, const gtsam::DiscreteKeys& keys,
                      const gtsam::DecisionTreeFactor::ADT& potentials);
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
  gtsam::DiscreteValues argmax(
      gtsam::DiscreteValues given = gtsam::DiscreteValues()) const;
};

#include <gtsam/discrete/DiscreteFactorGraph.h>
pair<gtsam::DiscreteConditional*, gtsam::DiscreteFactor*>
EliminateDiscrete(const gtsam::DiscreteFactorGraph& factors,
                  const gtsam::Ordering& frontalKeys);

pair<gtsam::DiscreteConditional*, gtsam::DiscreteFactor*>
EliminateForMPE(const gtsam::DiscreteFactorGraph& factors,
                const gtsam::Ordering& frontalKeys);

#include <gtsam/inference/EliminateableFactorGraph.h>
class DiscreteFactorGraph {
  std::shared_ptr<gtsam::DiscreteBayesNet> eliminateSequential(
      gtsam::DiscreteFactorGraph::OptionalOrderingType orderingType = std::nullopt,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::DiscreteFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  std::shared_ptr<gtsam::DiscreteBayesNet> eliminateSequential(
      const gtsam::Ordering& ordering,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::DiscreteFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  pair<std::shared_ptr<gtsam::DiscreteBayesNet>,
       std::shared_ptr<gtsam::DiscreteFactorGraph>>
  eliminatePartialSequential(
      const gtsam::Ordering& ordering,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::DiscreteFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  pair<std::shared_ptr<gtsam::DiscreteBayesNet>,
       std::shared_ptr<gtsam::DiscreteFactorGraph>>
  eliminatePartialSequential(
      const gtsam::KeyVector& variables,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::DiscreteFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  std::shared_ptr<gtsam::DiscreteBayesTree> eliminateMultifrontal(
      gtsam::DiscreteFactorGraph::OptionalOrderingType orderingType = std::nullopt,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::DiscreteFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  std::shared_ptr<gtsam::DiscreteBayesTree> eliminateMultifrontal(
      const gtsam::Ordering& ordering,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::DiscreteFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  pair<std::shared_ptr<gtsam::DiscreteBayesTree>,
       std::shared_ptr<gtsam::DiscreteFactorGraph>>
  eliminatePartialMultifrontal(
      const gtsam::Ordering& ordering,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::DiscreteFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;
  pair<std::shared_ptr<gtsam::DiscreteBayesTree>,
       std::shared_ptr<gtsam::DiscreteFactorGraph>>
  eliminatePartialMultifrontal(
      const gtsam::KeyVector& variables,
      const gtsam::DiscreteFactorGraph::Eliminate& function =
          gtsam::DiscreteFactorGraph::Eliminate(
              gtsam::DiscreteFactorGraph::EliminationTraitsType::DefaultEliminate),
      gtsam::DiscreteFactorGraph::OptionalVariableIndex variableIndex = std::nullopt)
      const;

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

  gtsam::DiscreteFactor* product() const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DiscreteValues optimize(
      gtsam::DiscreteFactorGraph::OptionalOrderingType orderingType = std::nullopt)
      const;

  gtsam::DiscreteBayesNet sumProduct(
      gtsam::DiscreteFactorGraph::OptionalOrderingType orderingType = std::nullopt) const;
  gtsam::DiscreteBayesNet sumProduct(
      const gtsam::Ordering& ordering) const;

  gtsam::DiscreteLookupDAG maxProduct(
      gtsam::DiscreteFactorGraph::OptionalOrderingType orderingType = std::nullopt) const;
  gtsam::DiscreteLookupDAG maxProduct(
      const gtsam::Ordering& ordering) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      const string& s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;

  string markdown(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
  string html(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const std::map<gtsam::Key, std::vector<std::string>>& names =
          std::map<gtsam::Key, std::vector<std::string>>()) const;
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

#include <gtsam/discrete/DiscreteSearch.h>
class DiscreteSearchSolution {
  double error;
  gtsam::DiscreteValues assignment;
  DiscreteSearchSolution(double error, const gtsam::DiscreteValues& assignment);
};

class DiscreteSearch {
  static gtsam::DiscreteSearch FromFactorGraph(
      const gtsam::DiscreteFactorGraph& factorGraph,
      const gtsam::Ordering& ordering, bool buildJunctionTree = false);

  DiscreteSearch(const gtsam::DiscreteEliminationTree& etree);
  DiscreteSearch(const gtsam::DiscreteJunctionTree& junctionTree);
  DiscreteSearch(const gtsam::DiscreteBayesNet& bayesNet);
  DiscreteSearch(const gtsam::DiscreteBayesTree& bayesTree);

  void print(string name = "DiscreteSearch: ",
             const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;

  double lowerBound() const;

  std::vector<gtsam::DiscreteSearchSolution> run(size_t K = 1) const;
};

#include <gtsam/discrete/DiscreteMarginals.h>

class DiscreteMarginals {
  DiscreteMarginals();
  DiscreteMarginals(const gtsam::DiscreteFactorGraph& graph);

  gtsam::DiscreteFactor* operator()(gtsam::Key variable) const;
  gtsam::Vector marginalProbabilities(const gtsam::DiscreteKey& key) const;

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

}  // namespace gtsam
