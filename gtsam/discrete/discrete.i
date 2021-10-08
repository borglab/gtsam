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

#include <gtsam/discrete/DiscreteFactor.h>
class DiscreteFactor {
};

#include <gtsam/discrete/Signature.h>
class Signature {
  Signature(gtsam::DiscreteKey key);
};

#include <gtsam/discrete/DecisionTreeFactor.h>
class DecisionTreeFactor: gtsam::DiscreteFactor {
  DecisionTreeFactor();
};

#include <gtsam/discrete/DiscreteFactorGraph.h>
class DiscreteFactorGraph {
  DiscreteFactorGraph();
  void add(const gtsam::DiscreteKey& j, string table);
  void add(const gtsam::DiscreteKeys& j, string table);
  void print(string s = "") const;
};

}  // namespace gtsam
