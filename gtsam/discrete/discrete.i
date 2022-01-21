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
};

// DiscreteValues is added in specializations/discrete.h as a std::map
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
class DiscreteFactor {
  void print(string s = "DiscreteFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol = 1e-9) const;
  bool empty() const;
  size_t size() const;
  double operator()(const gtsam::DiscreteValues& values) const;
};

#include <gtsam/discrete/DecisionTreeFactor.h>
virtual class DecisionTreeFactor : gtsam::DiscreteFactor {
  DecisionTreeFactor();
  
  DecisionTreeFactor(const gtsam::DiscreteKey& key,
                     const std::vector<double>& spec);
  DecisionTreeFactor(const gtsam::DiscreteKey& key, string table);
  
  DecisionTreeFactor(const gtsam::DiscreteKeys& keys, string table);
  DecisionTreeFactor(const std::vector<gtsam::DiscreteKey>& keys, string table);
  
  DecisionTreeFactor(const gtsam::DiscreteConditional& c);
  
  void print(string s = "DecisionTreeFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DecisionTreeFactor& other, double tol = 1e-9) const;

  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor operator*(const gtsam::DecisionTreeFactor& f) const;
  size_t cardinality(gtsam::Key j) const;
  gtsam::DecisionTreeFactor operator/(const gtsam::DecisionTreeFactor& f) const;
  gtsam::DecisionTreeFactor* sum(size_t nrFrontals) const;
  gtsam::DecisionTreeFactor* sum(const gtsam::Ordering& keys) const;
  gtsam::DecisionTreeFactor* max(size_t nrFrontals) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      bool showZero = true) const;
  std::vector<std::pair<DiscreteValues, double>> enumerate() const;
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
  gtsam::DiscreteConditional operator*(
      const gtsam::DiscreteConditional& other) const;
  DiscreteConditional marginal(gtsam::Key key) const;
  void print(string s = "Discrete Conditional\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteConditional& other, double tol = 1e-9) const;
  size_t nrFrontals() const;
  size_t nrParents() const;
  void printSignature(
      string s = "Discrete Conditional: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  gtsam::DecisionTreeFactor* choose(const gtsam::DiscreteValues& given) const;
  gtsam::DecisionTreeFactor* likelihood(
      const gtsam::DiscreteValues& frontalValues) const;
  gtsam::DecisionTreeFactor* likelihood(size_t value) const;
  size_t solve(const gtsam::DiscreteValues& parentsValues) const;
  size_t sample(const gtsam::DiscreteValues& parentsValues) const;
  size_t sample(size_t value) const;
  size_t sample() const;
  void solveInPlace(gtsam::DiscreteValues @parentsValues) const;
  void sampleInPlace(gtsam::DiscreteValues @parentsValues) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter =
                      gtsam::DefaultKeyFormatter) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter,
                  std::map<gtsam::Key, std::vector<std::string>> names) const;
  string html(const gtsam::KeyFormatter& keyFormatter =
                  gtsam::DefaultKeyFormatter) const;
  string html(const gtsam::KeyFormatter& keyFormatter,
              std::map<gtsam::Key, std::vector<std::string>> names) const;
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
  size_t solve() const;
  size_t argmax() const;
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
  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void saveGraph(string s, const gtsam::KeyFormatter& keyFormatter =
                               gtsam::DefaultKeyFormatter) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DiscreteValues optimize() const;
  gtsam::DiscreteValues optimize(gtsam::DiscreteValues given) const;
  gtsam::DiscreteValues sample() const;
  gtsam::DiscreteValues sample(gtsam::DiscreteValues given) const;
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
  void printSignature(
      const string& s = "Clique: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  double evaluate(const gtsam::DiscreteValues& values) const;
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

#include <gtsam/inference/DotWriter.h>
class DotWriter {
  DotWriter(double figureWidthInches = 5, double figureHeightInches = 5,
            bool plotFactorPoints = true, bool connectKeysToFactor = true,
            bool binaryEdges = true);
};

#include <gtsam/discrete/DiscreteFactorGraph.h>
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

  bool empty() const;
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::DiscreteFactor* at(size_t i) const;

  void print(string s = "") const;
  bool equals(const gtsam::DiscreteFactorGraph& fg, double tol = 1e-9) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& dotWriter = gtsam::DotWriter()) const;
  void saveGraph(
      string s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& dotWriter = gtsam::DotWriter()) const;

  gtsam::DecisionTreeFactor product() const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DiscreteValues optimize() const;

  gtsam::DiscreteBayesNet eliminateSequential();
  gtsam::DiscreteBayesNet eliminateSequential(const gtsam::Ordering& ordering);
  std::pair<gtsam::DiscreteBayesNet, gtsam::DiscreteFactorGraph>
      eliminatePartialSequential(const gtsam::Ordering& ordering);
  gtsam::DiscreteBayesTree eliminateMultifrontal();
  gtsam::DiscreteBayesTree eliminateMultifrontal(const gtsam::Ordering& ordering);
  std::pair<gtsam::DiscreteBayesTree, gtsam::DiscreteFactorGraph>
      eliminatePartialMultifrontal(const gtsam::Ordering& ordering);

  string markdown(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  string markdown(const gtsam::KeyFormatter& keyFormatter,
                  std::map<gtsam::Key, std::vector<std::string>> names) const;
  string html(const gtsam::KeyFormatter& keyFormatter =
                  gtsam::DefaultKeyFormatter) const;
  string html(const gtsam::KeyFormatter& keyFormatter,
              std::map<gtsam::Key, std::vector<std::string>> names) const;
};

}  // namespace gtsam
