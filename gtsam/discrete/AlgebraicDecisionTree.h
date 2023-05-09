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

#include <gtsam/discrete/DecisionTree-inl.h>

namespace gtsam {

  /**
   * Algebraic Decision Trees fix the range to double
   * Just has some nice constructors and some syntactic sugar
   * TODO: consider eliminating this class altogether?
   */
  template<typename L>
  class AlgebraicDecisionTree: public DecisionTree<L, double> {

  public:

    typedef DecisionTree<L, double> Super;

    /** The Real ring with addition and multiplication */
    struct Ring {
      static inline double zero() {
        return 0.0;
      }
      static inline double one() {
        return 1.0;
      }
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
      static inline double id(const double& x) {
        return x;
      }
    };

    AlgebraicDecisionTree() :
        Super(1.0) {
    }

    AlgebraicDecisionTree(const Super& add) :
        Super(add) {
    }

    /** Create a new leaf function splitting on a variable */
    AlgebraicDecisionTree(const L& label, double y1, double y2) :
        Super(label, y1, y2) {
    }

    /** Create a new leaf function splitting on a variable */
    AlgebraicDecisionTree(const typename Super::LabelC& labelC, double y1, double y2) :
        Super(labelC, y1, y2) {
    }

    /** Create from keys and vector table */
    AlgebraicDecisionTree //
    (const std::vector<typename Super::LabelC>& labelCs, const std::vector<double>& ys) {
      this->root_ = Super::create(labelCs.begin(), labelCs.end(), ys.begin(),
          ys.end());
    }

    /** Create from keys and string table */
    AlgebraicDecisionTree //
    (const std::vector<typename Super::LabelC>& labelCs, const std::string& table) {
      // Convert string to doubles
      std::vector<double> ys;
      std::istringstream iss(table);
      std::copy(std::istream_iterator<double>(iss),
          std::istream_iterator<double>(), std::back_inserter(ys));

      // now call recursive Create
      this->root_ = Super::create(labelCs.begin(), labelCs.end(), ys.begin(),
          ys.end());
    }

    /** Create a new function splitting on a variable */
    template<typename Iterator>
    AlgebraicDecisionTree(Iterator begin, Iterator end, const L& label) :
        Super(nullptr) {
      this->root_ = compose(begin, end, label);
    }

    /** Convert */
    template<typename M>
    AlgebraicDecisionTree(const AlgebraicDecisionTree<M>& other,
        const std::map<M, L>& map) {
      this->root_ = this->template convert<M, double>(other.root_, map,
          Ring::id);
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
    AlgebraicDecisionTree sum(const typename Super::LabelC& labelC) const {
      return this->combine(labelC, &Ring::add);
    }

  };
// AlgebraicDecisionTree

}
// namespace gtsam
