//*************************************************************************
// basis
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
  double value(const gtsam::DiscreteValues& values) const;
};

#include <gtsam/discrete/DiscreteConditional.h>
class DiscreteConditional {
  DiscreteConditional();
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
  double value(const gtsam::DiscreteValues& values) const; // TODO(dellaert): why do I have to repeat???
};

#include <gtsam/discrete/DiscreteFactorGraph.h>
class DiscreteFactorGraph {
  DiscreteFactorGraph();
  void add(const gtsam::DiscreteKey& j, string table);
  void add(const gtsam::DiscreteKey& j1, const gtsam::DiscreteKey& j2, string table);
  void add(const gtsam::DiscreteKeys& keys, string table);
  void print(string s = "") const;
  bool equals(const gtsam::DiscreteFactorGraph& fg, double tol = 1e-9) const;
  gtsam::KeySet keys() const;
  gtsam::DecisionTreeFactor product() const;
  double value(const gtsam::DiscreteValues& values) const;
  DiscreteValues optimize() const;
};

}  // namespace gtsam
