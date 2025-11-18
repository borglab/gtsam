//*************************************************************************
// inference
//*************************************************************************

namespace gtsam {

// Headers for overloaded methods below, break hierarchy :-/
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>

#include <gtsam/inference/Key.h>

// Default keyformatter
void PrintKeyList(
    const gtsam::KeyList& keys, const string& s = "",
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
void PrintKeyVector(
    const gtsam::KeyVector& keys, const string& s = "",
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
void PrintKeySet(
    const gtsam::KeySet& keys, const string& s = "",
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);

#include <gtsam/inference/Symbol.h>
class Symbol {
  Symbol();
  Symbol(char c, uint64_t j);
  Symbol(gtsam::Key key);

  gtsam::Key key() const;
  void print(const string& s = "") const;
  bool equals(const gtsam::Symbol& expected, double tol) const;

  char chr() const;
  uint64_t index() const;
  string string() const;
};

gtsam::Key symbol(char chr, size_t index);
char symbolChr(gtsam::Key key);
size_t symbolIndex(gtsam::Key key);

namespace symbol_shorthand {
gtsam::Key A(size_t j);
gtsam::Key B(size_t j);
gtsam::Key C(size_t j);
gtsam::Key D(size_t j);
gtsam::Key E(size_t j);
gtsam::Key F(size_t j);
gtsam::Key G(size_t j);
gtsam::Key H(size_t j);
gtsam::Key I(size_t j);
gtsam::Key J(size_t j);
gtsam::Key K(size_t j);
gtsam::Key L(size_t j);
gtsam::Key M(size_t j);
gtsam::Key N(size_t j);
gtsam::Key O(size_t j);
gtsam::Key P(size_t j);
gtsam::Key Q(size_t j);
gtsam::Key R(size_t j);
gtsam::Key S(size_t j);
gtsam::Key T(size_t j);
gtsam::Key U(size_t j);
gtsam::Key V(size_t j);
gtsam::Key W(size_t j);
gtsam::Key X(size_t j);
gtsam::Key Y(size_t j);
gtsam::Key Z(size_t j);
}  // namespace symbol_shorthand

#include <gtsam/inference/LabeledSymbol.h>
class LabeledSymbol {
  LabeledSymbol(gtsam::Key full_key);
  LabeledSymbol(const gtsam::LabeledSymbol& key);
  LabeledSymbol(unsigned char valType, unsigned char label, size_t j);

  gtsam::Key key() const;
  unsigned char label() const;
  unsigned char chr() const;
  size_t index() const;

  gtsam::LabeledSymbol upper() const;
  gtsam::LabeledSymbol lower() const;
  gtsam::LabeledSymbol newChr(unsigned char c) const;
  gtsam::LabeledSymbol newLabel(unsigned char label) const;

  void print(string s = "") const;
};

gtsam::Key mrsymbol(unsigned char c, unsigned char label, size_t j);
unsigned char mrsymbolChr(gtsam::Key key);
unsigned char mrsymbolLabel(gtsam::Key key);
size_t mrsymbolIndex(gtsam::Key key);

#include <gtsam/inference/EdgeKey.h>
class EdgeKey {
  EdgeKey(std::uint32_t i, std::uint32_t j);
  EdgeKey(gtsam::Key key);
  EdgeKey(const gtsam::EdgeKey& key);

  std::uint32_t i() const;
  std::uint32_t j() const;
  gtsam::Key key() const;

  void print(string s = "") const;
};

#include <gtsam/inference/Ordering.h>
class Ordering {
  /// Type of ordering to use
  enum OrderingType { COLAMD, METIS, NATURAL, CUSTOM };

  // Standard Constructors and Named Constructors
  Ordering();
  Ordering(const gtsam::Ordering& other);
  Ordering(const std::vector<size_t>& keys);

  template <
      FACTOR_GRAPH = {gtsam::NonlinearFactorGraph, gtsam::DiscreteFactorGraph,
                      gtsam::SymbolicFactorGraph, gtsam::GaussianFactorGraph, gtsam::HybridGaussianFactorGraph}>
                      
  static gtsam::Ordering Colamd(const FACTOR_GRAPH& graph);
  static gtsam::Ordering Colamd(const gtsam::VariableIndex& variableIndex);

  template <
      FACTOR_GRAPH = {gtsam::NonlinearFactorGraph, gtsam::DiscreteFactorGraph,
                      gtsam::SymbolicFactorGraph, gtsam::GaussianFactorGraph, gtsam::HybridGaussianFactorGraph}>
  static gtsam::Ordering ColamdConstrainedLast(
      const FACTOR_GRAPH& graph, const gtsam::KeyVector& constrainLast,
      bool forceOrder = false);

  template <
      FACTOR_GRAPH = {gtsam::NonlinearFactorGraph, gtsam::DiscreteFactorGraph,
                      gtsam::SymbolicFactorGraph, gtsam::GaussianFactorGraph, gtsam::HybridGaussianFactorGraph}>
  static gtsam::Ordering ColamdConstrainedFirst(
      const FACTOR_GRAPH& graph, const gtsam::KeyVector& constrainFirst,
      bool forceOrder = false);

  template <
      FACTOR_GRAPH = {gtsam::NonlinearFactorGraph, gtsam::DiscreteFactorGraph,
                      gtsam::SymbolicFactorGraph, gtsam::GaussianFactorGraph, gtsam::HybridGaussianFactorGraph}>
  static gtsam::Ordering Natural(const FACTOR_GRAPH& fg);

  template <
      FACTOR_GRAPH = {gtsam::NonlinearFactorGraph, gtsam::DiscreteFactorGraph,
                      gtsam::SymbolicFactorGraph, gtsam::GaussianFactorGraph, gtsam::HybridGaussianFactorGraph}>
  static gtsam::Ordering Metis(const FACTOR_GRAPH& graph);

  template <
      FACTOR_GRAPH = {gtsam::NonlinearFactorGraph, gtsam::DiscreteFactorGraph,
                      gtsam::SymbolicFactorGraph, gtsam::GaussianFactorGraph, gtsam::HybridGaussianFactorGraph}>
  static gtsam::Ordering Create(gtsam::Ordering::OrderingType orderingType,
                                const FACTOR_GRAPH& graph);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::Ordering& other, double tol) const;

  // Standard interface
  size_t size() const;
  size_t at(size_t i) const;
  void push_back(gtsam::Key key);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/inference/DotWriter.h>
class DotWriter {
  DotWriter(double figureWidthInches = 5, double figureHeightInches = 5,
            bool plotFactorPoints = true, bool connectKeysToFactor = true,
            bool binaryEdges = true);

  double figureWidthInches;
  double figureHeightInches;
  bool plotFactorPoints;
  bool connectKeysToFactor;
  bool binaryEdges;

  std::map<gtsam::Key, gtsam::Vector2> variablePositions;
  std::map<char, double> positionHints;
  std::set<gtsam::Key> boxes;
  std::map<size_t, gtsam::Vector2> factorPositions;
};

#include <gtsam/inference/VariableIndex.h>
class VariableIndex {
  // Standard Constructors and Named Constructors
  VariableIndex();
  template <T = {gtsam::SymbolicFactorGraph, gtsam::GaussianFactorGraph,
                 gtsam::NonlinearFactorGraph}>
  VariableIndex(const T& factorGraph);
  VariableIndex(const gtsam::VariableIndex& other);

  gtsam::FactorIndices& at(gtsam::Key variable) const;
  bool empty(gtsam::Key variable) const;

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

#include <gtsam/inference/Factor.h>
virtual class Factor {
  void print(string s = "Factor\n", const gtsam::KeyFormatter& keyFormatter =
                                        gtsam::DefaultKeyFormatter) const;
  void printKeys(string s = "") const;
  bool equals(const gtsam::Factor& other, double tol = 1e-9) const;
  bool empty() const;
  size_t size() const;
  gtsam::KeyVector keys() const;
};

}  // namespace gtsam
