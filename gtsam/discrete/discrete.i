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
virtual class DecisionTreeFactor: gtsam::DiscreteFactor {
  DecisionTreeFactor();
  DecisionTreeFactor(const gtsam::DiscreteKeys& keys, string table);
  DecisionTreeFactor(const gtsam::DiscreteConditional& c);
  void print(string s = "DecisionTreeFactor\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DecisionTreeFactor& other, double tol = 1e-9) const;
  string dot(bool showZero = false) const;
};

#include <gtsam/discrete/DiscreteConditional.h>
virtual class DiscreteConditional : gtsam::DecisionTreeFactor {
  DiscreteConditional();
  DiscreteConditional(size_t nFrontals, const gtsam::DecisionTreeFactor& f);
  DiscreteConditional(const gtsam::DiscreteKey& key,
                      const gtsam::DiscreteKeys& parents, string spec);
  DiscreteConditional(const gtsam::DecisionTreeFactor& joint,
                      const gtsam::DecisionTreeFactor& marginal);
  DiscreteConditional(const gtsam::DecisionTreeFactor& joint,
                      const gtsam::DecisionTreeFactor& marginal,
                      const gtsam::Ordering& orderedKeys);
  void print(string s = "Discrete Conditional\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteConditional& other, double tol = 1e-9) const;
  void printSignature(
      string s = "Discrete Conditional: ",
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const;
  gtsam::DecisionTreeFactor* toFactor() const;
  gtsam::DecisionTreeFactor* chooseAsFactor(const gtsam::DiscreteValues& parentsValues) const;
  size_t solve(const gtsam::DiscreteValues& parentsValues) const;
  size_t sample(const gtsam::DiscreteValues& parentsValues) const;
  void solveInPlace(gtsam::DiscreteValues@ parentsValues) const;
  void sampleInPlace(gtsam::DiscreteValues@ parentsValues) const;
};

#include <gtsam/discrete/DiscreteBayesNet.h>
class DiscreteBayesNet { 
  DiscreteBayesNet();
  void add(const gtsam::DiscreteKey& key,
           const gtsam::DiscreteKeys& parents, string spec);
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
  void saveGraph(string s,
                const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void add(const gtsam::DiscreteConditional& s);
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DiscreteValues optimize() const;
  gtsam::DiscreteValues sample() const;
};

#include <gtsam/discrete/DiscreteBayesTree.h>
class DiscreteBayesTree {
  DiscreteBayesTree();
  void print(string s = "DiscreteBayesTree\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteBayesTree& other, double tol = 1e-9) const;
  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void saveGraph(string s,
                const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  double operator()(const gtsam::DiscreteValues& values) const;
};

#include <gtsam/inference/DotWriter.h>
class DotWriter {
  DotWriter();
};

#include <gtsam/discrete/DiscreteFactorGraph.h>
class DiscreteFactorGraph {
  DiscreteFactorGraph();
  DiscreteFactorGraph(const gtsam::DiscreteBayesNet& bayesNet);
  
  void add(const gtsam::DiscreteKey& j, string table);
  void add(const gtsam::DiscreteKey& j1, const gtsam::DiscreteKey& j2, string table);
  void add(const gtsam::DiscreteKeys& keys, string table);
  
  bool empty() const;
  size_t size() const;
  gtsam::KeySet keys() const;
  const gtsam::DiscreteFactor* at(size_t i) const;

  void print(string s = "") const;
  bool equals(const gtsam::DiscreteFactorGraph& fg, double tol = 1e-9) const;

  string dot(const gtsam::DotWriter& dotWriter = gtsam::DotWriter(),
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void saveGraph(string s,
                 const gtsam::DotWriter& dotWriter = gtsam::DotWriter(),
                 const gtsam::KeyFormatter& keyFormatter =
                     gtsam::DefaultKeyFormatter) const;

  gtsam::DecisionTreeFactor product() const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DiscreteValues optimize() const;

  gtsam::DiscreteBayesNet eliminateSequential();
  gtsam::DiscreteBayesNet eliminateSequential(const gtsam::Ordering& ordering);
  gtsam::DiscreteBayesTree eliminateMultifrontal();
  gtsam::DiscreteBayesTree eliminateMultifrontal(const gtsam::Ordering& ordering);
};

}  // namespace gtsam
