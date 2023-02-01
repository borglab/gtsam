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
#include <vector>
namespace gtsam {

  /**
   * Algebraic Decision Trees fix the range to double
   * Just has some nice constructors and some syntactic sugar
   * TODO: consider eliminating this class altogether?
   *
   * @ingroup discrete
   */
  template <typename L>
  class GTSAM_EXPORT AlgebraicDecisionTree : public DecisionTree<L, double> {
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
    };

    AlgebraicDecisionTree(double leaf = 1.0) : Base(leaf) {}

    // Explicitly non-explicit constructor
    AlgebraicDecisionTree(const Base& add) : Base(add) {}

    /** Create a new leaf function splitting on a variable */
    AlgebraicDecisionTree(const L& label, double y1, double y2)
        : Base(label, y1, y2) {}

    /** Create a new leaf function splitting on a variable */
    AlgebraicDecisionTree(const typename Base::LabelC& labelC, double y1,
                          double y2)
        : Base(labelC, y1, y2) {}

    /** Create from keys and vector table */
    AlgebraicDecisionTree  //
        (const std::vector<typename Base::LabelC>& labelCs,
        const std::vector<double>& ys) {
      this->root_ =
          Base::create(labelCs.begin(), labelCs.end(), ys.begin(), ys.end());
    }

    /** Create from keys and string table */
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

    /** Create a new function splitting on a variable */
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

    /** sum */
    AlgebraicDecisionTree operator+(const AlgebraicDecisionTree& g) const {
      return this->apply(g, &Ring::add);
    }

    /** product */
    AlgebraicDecisionTree operator*(const AlgebraicDecisionTree& g) const {
      return this->apply(g, &Ring::mul);
    }

    /** division */
    AlgebraicDecisionTree operator/(const AlgebraicDecisionTree& g) const {
      return this->apply(g, &Ring::div);
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
        return (boost::format("%4.8g") % v).str();
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
