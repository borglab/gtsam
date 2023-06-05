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
  Symbol(size_t key);

  size_t key() const;
  void print(const string& s = "") const;
  bool equals(const gtsam::Symbol& expected, double tol) const;

  char chr() const;
  uint64_t index() const;
  string string() const;
};

size_t symbol(char chr, size_t index);
char symbolChr(size_t key);
size_t symbolIndex(size_t key);

namespace symbol_shorthand {
size_t A(size_t j);
size_t B(size_t j);
size_t C(size_t j);
size_t D(size_t j);
size_t E(size_t j);
size_t F(size_t j);
size_t G(size_t j);
size_t H(size_t j);
size_t I(size_t j);
size_t J(size_t j);
size_t K(size_t j);
size_t L(size_t j);
size_t M(size_t j);
size_t N(size_t j);
size_t O(size_t j);
size_t P(size_t j);
size_t Q(size_t j);
size_t R(size_t j);
size_t S(size_t j);
size_t T(size_t j);
size_t U(size_t j);
size_t V(size_t j);
size_t W(size_t j);
size_t X(size_t j);
size_t Y(size_t j);
size_t Z(size_t j);
}  // namespace symbol_shorthand

#include <gtsam/inference/LabeledSymbol.h>
class LabeledSymbol {
  LabeledSymbol(size_t full_key);
  LabeledSymbol(const gtsam::LabeledSymbol& key);
  LabeledSymbol(unsigned char valType, unsigned char label, size_t j);

  size_t key() const;
  unsigned char label() const;
  unsigned char chr() const;
  size_t index() const;

  gtsam::LabeledSymbol upper() const;
  gtsam::LabeledSymbol lower() const;
  gtsam::LabeledSymbol newChr(unsigned char c) const;
  gtsam::LabeledSymbol newLabel(unsigned char label) const;

  void print(string s = "") const;
};

size_t mrsymbol(unsigned char c, unsigned char label, size_t j);
unsigned char mrsymbolChr(size_t key);
unsigned char mrsymbolLabel(size_t key);
size_t mrsymbolIndex(size_t key);

#include <gtsam/inference/Ordering.h>
class Ordering {
  /// Type of ordering to use
  enum OrderingType { COLAMD, METIS, NATURAL, CUSTOM };

  // Standard Constructors and Named Constructors
  Ordering();
  Ordering(const gtsam::Ordering& other);

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
  static gtsam::Ordering Natural(const FACTOR_GRAPH& graph);

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
  bool equals(const gtsam::Ordering& ord, double tol) const;

  // Standard interface
  size_t size() const;
  size_t at(size_t i) const;
  void push_back(size_t key);

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
