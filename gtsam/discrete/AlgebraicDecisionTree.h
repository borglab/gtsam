/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AlgebraicDecisionTree.h
 * @brief    Algebraic Decision Trees
 * @author  Frank Dellaert
 * @date  Mar 14, 2011
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTree-inl.h>

#include <algorithm>
#include <map>
#include <string>
#include <iomanip>
#include <vector>
namespace gtsam {

  /**
   * An algebraic decision tree fixes the range of a DecisionTree to double.
   * Just has some nice constructors and some syntactic sugar.
   * TODO(dellaert): consider eliminating this class altogether?
   *
   * @ingroup discrete
   */
  template <typename L>
  class AlgebraicDecisionTree : public DecisionTree<L, double> {
    /**
     * @brief Default method used by `labelFormatter` or `valueFormatter` when
     * printing.
     *
     * @param x The value passed to format.
     * @return std::string
     */
    static std::string DefaultFormatter(const L& x) {
      std::stringstream ss;
      ss << x;
      return ss.str();
    }

   public:
    using Base = DecisionTree<L, double>;

    /** The Real ring with addition and multiplication */
    struct Ring {
      static inline double zero() { return 0.0; }
      static inline double one() { return 1.0; }
      static inline double add(const double& a, const double& b) {
        return a + b;
      }
      static inline double max(const double& a, const double& b) {
        return std::max(a, b);
      }
      static inline double mul(const double& a, const double& b) {
        return a * b;
      }
      static inline double div(const double& a, const double& b) {
        return a / b;
      }
      static inline double id(const double& x) { return x; }
      static inline double negate(const double& x) { return -x; }
    };

    AlgebraicDecisionTree(double leaf = 1.0) : Base(leaf) {}

    // Explicitly non-explicit constructor
    AlgebraicDecisionTree(const Base& add) : Base(add) {}

    /** Create a new leaf function splitting on a variable */
    AlgebraicDecisionTree(const L& label, double y1, double y2)
        : Base(label, y1, y2) {}

    /**
     * @brief Create a new leaf function splitting on a variable
     *
     * @param labelC: The label with cardinality 2
     * @param y1: The value for the first key
     * @param y2: The value for the second key
     *
     * Example:
     * @code{.cpp}
     * std::pair<string, size_t> A {"a", 2};
     * AlgebraicDecisionTree<string> a(A, 0.6, 0.4);
     * @endcode
     */
    AlgebraicDecisionTree(const typename Base::LabelC& labelC, double y1,
                          double y2)
        : Base(labelC, y1, y2) {}

    /** 
     * @brief Create from keys with cardinalities and a vector table
     * 
     * @param labelCs: The keys, with cardinalities, given as pairs
     * @param ys: The vector table
     *
     * Example with three keys, A, B, and C, with cardinalities 2, 3, and 2,
     * respectively, and a vector table of size 12:
     * @code{.cpp}
     * DiscreteKey A(0, 2), B(1, 3), C(2, 2);
     * const vector<double> cpt{
     *   1.0 / 3, 2.0 / 3, 3.0 / 7, 4.0 / 7, 5.0 / 11, 6.0 / 11,  //
     *   1.0 / 9, 8.0 / 9, 3.0 / 6, 3.0 / 6, 5.0 / 10, 5.0 / 10};
     * AlgebraicDecisionTree<Key> expected(A & B & C, cpt);
     * @endcode
     * The table is given in the following order:
     *   A=0, B=0, C=0
     *   A=0, B=0, C=1
     *   ...
     *   A=1, B=1, C=1
     * Hence, the first line in the table is for A==0, and the second for A==1.
     * In each line, the first two entries are for B==0, the next two for B==1,
     * and the last two for B==2. Each pair is for a C value of 0 and 1.
     */
    AlgebraicDecisionTree  //
        (const std::vector<typename Base::LabelC>& labelCs,
         const std::vector<double>& ys) {
      this->root_ =
          Base::create(labelCs.begin(), labelCs.end(), ys.begin(), ys.end());
    }

    /** 
     * @brief Create from keys and string table
     * 
     * @param labelCs: The keys, with cardinalities, given as pairs
     * @param table: The string table, given as a string of doubles.
     * 
     * @note Table needs to be in same order as the vector table in the other constructor.
     */
    AlgebraicDecisionTree  //
        (const std::vector<typename Base::LabelC>& labelCs,
        const std::string& table) {
      // Convert string to doubles
      std::vector<double> ys;
      std::istringstream iss(table);
      std::copy(std::istream_iterator<double>(iss),
                std::istream_iterator<double>(), std::back_inserter(ys));

      // now call recursive Create
      this->root_ =
          Base::create(labelCs.begin(), labelCs.end(), ys.begin(), ys.end());
    }

    /** 
     * @brief Create a range of decision trees, splitting on a single variable.
     * 
     * @param begin: Iterator to beginning of a range of decision trees
     * @param end: Iterator to end of a range of decision trees
     * @param label: The label to split on
     */
    template <typename Iterator>
    AlgebraicDecisionTree(Iterator begin, Iterator end, const L& label)
        : Base(nullptr) {
      this->root_ = compose(begin, end, label);
    }

    /**
     * Convert labels from type M to type L.
     *
     * @param other: The AlgebraicDecisionTree with label type M to convert.
     * @param map: Map from label type M to label type L.
     */
    template <typename M>
    AlgebraicDecisionTree(const AlgebraicDecisionTree<M>& other,
                          const std::map<M, L>& map) {
      // Functor for label conversion so we can use `convertFrom`.
      std::function<L(const M&)> L_of_M = [&map](const M& label) -> L {
        return map.at(label);
      };
      std::function<double(const double&)> op = Ring::id;
      this->root_ = DecisionTree<L, double>::convertFrom(other.root_, L_of_M, op);
    }

    /**
     * @brief Create from an arbitrary DecisionTree<L, X> by operating on it
     * with a functional `f`.
     *
     * @tparam X The type of the leaf of the original DecisionTree
     * @tparam Func Type signature of functional `f`.
     * @param other The original DecisionTree from which the
     * AlgbraicDecisionTree is constructed.
     * @param f Functional used to operate on
     * the leaves of the input DecisionTree.
     */
    template <typename X, typename Func>
    AlgebraicDecisionTree(const DecisionTree<L, X>& other, Func f)
        : Base(other, f) {}

    /** sum */
    AlgebraicDecisionTree operator+(const AlgebraicDecisionTree& g) const {
      return this->apply(g, &Ring::add);
    }

    /** negation */
    AlgebraicDecisionTree operator-() const {
      return this->apply(&Ring::negate);
    }

    /** subtract */
    AlgebraicDecisionTree operator-(const AlgebraicDecisionTree& g) const {
      return *this + (-g);
    }

    /** product */
    AlgebraicDecisionTree operator*(const AlgebraicDecisionTree& g) const {
      return this->apply(g, &Ring::mul);
    }

    /** division */
    AlgebraicDecisionTree operator/(const AlgebraicDecisionTree& g) const {
      return this->apply(g, &Ring::div);
    }

    /// Compute sum of all values
    double sum() const {
      double sum = 0;
      auto visitor = [&](double y) { sum += y; };
      this->visit(visitor);
      return sum;
    }

    /**
     * @brief Helper method to perform normalization such that all leaves in the
     * tree sum to 1
     *
     * @param sum
     * @return AlgebraicDecisionTree
     */
    AlgebraicDecisionTree normalize(double sum) const {
      return this->apply([&sum](const double& x) { return x / sum; });
    }

    /// Find the minimum values amongst all leaves
    double min() const {
      double min = std::numeric_limits<double>::max();
      auto visitor = [&](double x) { min = x < min ? x : min; };
      this->visit(visitor);
      return min;
    }

    /// Find the maximum values amongst all leaves
    double max() const {
      // Get the most negative value
      double max = -std::numeric_limits<double>::max();
      auto visitor = [&](double x) { max = x > max ? x : max; };
      this->visit(visitor);
      return max;
    }

    /** sum out variable */
    AlgebraicDecisionTree sum(const L& label, size_t cardinality) const {
      return this->combine(label, cardinality, &Ring::add);
    }

    /** sum out variable */
    AlgebraicDecisionTree sum(const typename Base::LabelC& labelC) const {
      return this->combine(labelC, &Ring::add);
    }

    /// print method customized to value type `double`.
    void print(const std::string& s = "",
               const typename Base::LabelFormatter& labelFormatter =
                   &DefaultFormatter) const {
      auto valueFormatter = [](const double& v) {
        std::stringstream ss;
        ss << std::setw(4) << std::setprecision(8) << v;
        return ss.str();
      };
      Base::print(s, labelFormatter, valueFormatter);
    }

    /// Equality method customized to value type `double`.
    bool equals(const AlgebraicDecisionTree& other, double tol = 1e-9) const {
      // lambda for comparison of two doubles upto some tolerance.
      auto compare = [tol](double a, double b) {
        return std::abs(a - b) < tol;
      };
      return Base::equals(other, compare);
    }
  };

template <typename T>
struct traits<AlgebraicDecisionTree<T>>
    : public Testable<AlgebraicDecisionTree<T>> {};
}  // namespace gtsam
