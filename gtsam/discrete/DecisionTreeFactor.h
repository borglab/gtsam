/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DecisionTreeFactor.h
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/inference/Ordering.h>

#include <algorithm>
#include <memory>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

  class DiscreteConditional;
  class HybridValues;

  /**
   * A discrete probabilistic factor.
   *
   * @ingroup discrete
   */
  class GTSAM_EXPORT DecisionTreeFactor : public DiscreteFactor,
                                          public AlgebraicDecisionTree<Key> {
   public:
    // typedefs needed to play nice with gtsam
    typedef DecisionTreeFactor This;
    typedef DiscreteFactor Base;  ///< Typedef to base class
    typedef std::shared_ptr<DecisionTreeFactor> shared_ptr;
    typedef AlgebraicDecisionTree<Key> ADT;

    /// @name Standard Constructors
    /// @{

    /** Default constructor for I/O */
    DecisionTreeFactor();

    /** Constructor from DiscreteKeys and AlgebraicDecisionTree */
    DecisionTreeFactor(const DiscreteKeys& keys, const ADT& potentials);

    /**
     * @brief Constructor from doubles
     *
     * @param keys The discrete keys.
     * @param table The table of values.
     *
     * @throw std::invalid_argument if the size of `table` does not match the
     * number of assignments.
     *
     * Example:
     * @code{.cpp}
     * DiscreteKey X(0,2), Y(1,3);
     * const std::vector<double> table {2, 5, 3, 6, 4, 7};
     * DecisionTreeFactor f1({X, Y}, table);
     * @endcode
     *
     * The values in the table should be laid out so that the first key varies
     * the slowest, and the last key the fastest.
     */
    DecisionTreeFactor(const DiscreteKeys& keys,
                       const std::vector<double>& table);

    /**
     * @brief Constructor from string
     *
     * @param keys The discrete keys.
     * @param table The table of values.
     *
     * @throw std::invalid_argument if the size of `table` does not match the
     * number of assignments.
     *
     * Example:
     * @code{.cpp}
     * DiscreteKey X(0,2), Y(1,3);
     * DecisionTreeFactor factor({X, Y}, "2 5 3 6 4 7");
     * @endcode
     *
     * The values in the table should be laid out so that the first key varies
     * the slowest, and the last key the fastest.
     */
    DecisionTreeFactor(const DiscreteKeys& keys, const std::string& table);

    /// Single-key specialization
    template <class SOURCE>
    DecisionTreeFactor(const DiscreteKey& key, SOURCE table)
        : DecisionTreeFactor(DiscreteKeys{key}, table) {}

    /// Single-key specialization, with vector of doubles.
    DecisionTreeFactor(const DiscreteKey& key, const std::vector<double>& row)
        : DecisionTreeFactor(DiscreteKeys{key}, row) {}

    /** Construct from a DiscreteConditional type */
    explicit DecisionTreeFactor(const DiscreteConditional& c);

    /// @}
    /// @name Testable
    /// @{

    /// equality
    bool equals(const DiscreteFactor& other, double tol = 1e-9) const override;

    // print
    void print(
        const std::string& s = "DecisionTreeFactor:\n",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override;

    /// @}
    /// @name Standard Interface
    /// @{

    /// Calculate probability for given values `x`, 
    /// is just look up in AlgebraicDecisionTree.
    double evaluate(const DiscreteValues& values) const  {
      return ADT::operator()(values);
    }

    /// Evaluate probability distribution, sugar.
    double operator()(const DiscreteValues& values) const override {
      return ADT::operator()(values);
    }

    /// Calculate error for DiscreteValues `x`, is -log(probability).
    double error(const DiscreteValues& values) const;

    /// multiply two factors
    DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override {
      return apply(f, ADT::Ring::mul);
    }

    static double safe_div(const double& a, const double& b);

    /// divide by factor f (safely)
    DecisionTreeFactor operator/(const DecisionTreeFactor& f) const {
      return apply(f, safe_div);
    }

    /// Convert into a decisiontree
    DecisionTreeFactor toDecisionTreeFactor() const override { return *this; }

    /// Create new factor by summing all values with the same separator values
    shared_ptr sum(size_t nrFrontals) const {
      return combine(nrFrontals, ADT::Ring::add);
    }

    /// Create new factor by summing all values with the same separator values
    shared_ptr sum(const Ordering& keys) const {
      return combine(keys, ADT::Ring::add);
    }

    /// Create new factor by maximizing over all values with the same separator.
    shared_ptr max(size_t nrFrontals) const {
      return combine(nrFrontals, ADT::Ring::max);
    }

    /// Create new factor by maximizing over all values with the same separator.
    shared_ptr max(const Ordering& keys) const {
      return combine(keys, ADT::Ring::max);
    }

    /// @}
    /// @name Advanced Interface
    /// @{

    /**
     * Apply unary operator (*this) "op" f
     * @param op a unary operator that operates on AlgebraicDecisionTree
     */
    DecisionTreeFactor apply(ADT::Unary op) const;

    /**
     * Apply unary operator (*this) "op" f
     * @param op a unary operator that operates on AlgebraicDecisionTree. Takes
     * both the assignment and the value.
     */
    DecisionTreeFactor apply(ADT::UnaryAssignment op) const;

    /**
     * Apply binary operator (*this) "op" f
     * @param f the second argument for op
     * @param op a binary operator that operates on AlgebraicDecisionTree
     */
    DecisionTreeFactor apply(const DecisionTreeFactor& f, ADT::Binary op) const;

    /**
     * Combine frontal variables using binary operator "op"
     * @param nrFrontals nr. of frontal to combine variables in this factor
     * @param op a binary operator that operates on AlgebraicDecisionTree
     * @return shared pointer to newly created DecisionTreeFactor
     */
    shared_ptr combine(size_t nrFrontals, ADT::Binary op) const;

    /**
     * Combine frontal variables in an Ordering using binary operator "op"
     * @param nrFrontals nr. of frontal to combine variables in this factor
     * @param op a binary operator that operates on AlgebraicDecisionTree
     * @return shared pointer to newly created DecisionTreeFactor
     */
    shared_ptr combine(const Ordering& keys, ADT::Binary op) const;

    /// Enumerate all values into a map from values to double.
    std::vector<std::pair<DiscreteValues, double>> enumerate() const;

    /// Get all the probabilities in order of assignment values
    std::vector<double> probabilities() const;

    /**
     * @brief Prune the decision tree of discrete variables.
     *
     * Pruning will set the leaves to be "pruned" to 0 indicating a 0
     * probability. An assignment is pruned if it is not in the top
     * `maxNrAssignments` values.
     *
     * A violation can occur if there are more
     * duplicate values than `maxNrAssignments`. A violation here is the need to
     * un-prune the decision tree (e.g. all assignment values are 1.0). We could
     * have another case where some subset of duplicates exist (e.g. for a tree
     * with 8 assignments we have 1, 1, 1, 1, 0.8, 0.7, 0.6, 0.5), but this is
     * not a violation since the for `maxNrAssignments=5` the top values are (1,
     * 0.8).
     *
     * @param maxNrAssignments The maximum number of assignments to keep.
     * @return DecisionTreeFactor
     */
    DecisionTreeFactor prune(size_t maxNrAssignments) const;

    /// @}
    /// @name Wrapper support
    /// @{

    /** output to graphviz format, stream version */
    void dot(std::ostream& os,
            const KeyFormatter& keyFormatter = DefaultKeyFormatter,
            bool showZero = true) const;

    /** output to graphviz format, open a file */
    void dot(const std::string& name,
            const KeyFormatter& keyFormatter = DefaultKeyFormatter,
            bool showZero = true) const;

    /** output to graphviz format string */
    std::string dot(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                    bool showZero = true) const;

    /**
     * @brief Render as markdown table
     *
     * @param keyFormatter GTSAM-style Key formatter.
     * @param names optional, category names corresponding to choices.
     * @return std::string a markdown string.
     */
    std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                        const Names& names = {}) const override;

    /**
     * @brief Render as html table
     *
     * @param keyFormatter GTSAM-style Key formatter.
     * @param names optional, category names corresponding to choices.
     * @return std::string a html string.
     */
    std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                    const Names& names = {}) const override;

  /// @}
  /// @name HybridValues methods.
  /// @{

  /**
   * Calculate error for HybridValues `x`, is -log(probability)
   * Simply dispatches to DiscreteValues version.
   */
  double error(const HybridValues& values) const override;

  /// Compute error for each assignment and return as a tree
  AlgebraicDecisionTree<Key> errorTree() const override;

  /// @}

   private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
      ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(ADT);
    }
#endif
  };

// traits
template <>
struct traits<DecisionTreeFactor> : public Testable<DecisionTreeFactor> {};
}  // namespace gtsam
