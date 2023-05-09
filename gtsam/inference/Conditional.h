/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Conditional.h
 * @brief   Base class for conditional densities
 * @author  Frank Dellaert
 */

// \callgraph
#pragma once

#include <gtsam/inference/Key.h>

namespace gtsam {

  class HybridValues;  // forward declaration.

  /**
   * This is the base class for all conditional distributions/densities, 
   * which are implemented as specialized factors.  This class does not store any
   * data other than its keys.  Derived classes store data such as matrices and
   * probability tables.
   *
   * The `evaluate` method is used to evaluate the factor, and together with 
   * `logProbability` is the main methods that need to be implemented in derived
   * classes. These two methods relate to the `error` method in the factor by:
   *   probability(x) = k exp(-error(x))
   * where k is a normalization constant making \int probability(x) == 1.0, and
   *   logProbability(x) = K - error(x)
   * i.e., K = log(K). The normalization constant K is assumed to *not* depend
   * on any argument, only (possibly) on the conditional parameters.
   * This class provides a default logNormalizationConstant() == 0.0.
   * 
   * There are four broad classes of conditionals that derive from Conditional:
   *
   * - \b Gaussian conditionals, implemented in \class GaussianConditional, a 
   *   Gaussian density over a set of continuous variables.
   * - \b Discrete conditionals, implemented in \class DiscreteConditional, which
   *   represent a discrete conditional distribution over discrete variables.
   * - \b Hybrid conditional densities, such as \class GaussianMixture, which is 
   *   a density over continuous variables given discrete/continuous parents.
   * - \b Symbolic factors, used to represent a graph structure, implemented in 
   *   \class SymbolicConditional. Only used for symbolic elimination etc.
   *
   * Derived classes *must* redefine the Factor and shared_ptr typedefs to refer
   * to the associated factor type and shared_ptr type of the derived class.  See
   * SymbolicConditional and GaussianConditional for examples.
   * 
   * \nosubgrouping
   */
  template<class FACTOR, class DERIVEDCONDITIONAL>
  class Conditional
  {
  protected:
    /** The first nrFrontal variables are frontal and the rest are parents. */
    size_t nrFrontals_;

  private:
    /// Typedef to this class
    typedef Conditional<FACTOR,DERIVEDCONDITIONAL> This;


  public:
    /** A mini implementation of an iterator range, to share const
     * views of frontals and parents. */
    typedef std::pair<typename FACTOR::const_iterator, typename FACTOR::const_iterator> ConstFactorRange;
    struct ConstFactorRangeIterator {
      ConstFactorRange range_;
      // Delete default constructor
      ConstFactorRangeIterator() = delete;
      ConstFactorRangeIterator(ConstFactorRange const& x) : range_(x) {}
      // Implement begin and end for iteration
      typename FACTOR::const_iterator begin() const { return range_.first; }
      typename FACTOR::const_iterator end() const { return range_.second; }
      size_t size() const { return std::distance(range_.first, range_.second); }
      const auto& front() const { return *begin(); }
      // == operator overload for comparison with another iterator
      template<class OTHER>
      bool operator==(const OTHER& rhs) const {
        return std::equal(begin(), end(), rhs.begin());
      }
    };

    /** View of the frontal keys (call frontals()) */
    typedef ConstFactorRangeIterator Frontals;

    /** View of the separator keys (call parents()) */
    typedef ConstFactorRangeIterator Parents;

  protected:
    /// @name Standard Constructors
    /// @{

    /** Empty Constructor to make serialization possible */
    Conditional() : nrFrontals_(0) {}

    /** Constructor */
    Conditional(size_t nrFrontals) : nrFrontals_(nrFrontals) {}

    /// @}

  public:
    /// @name Testable
    /// @{

    /** print with optional formatter */
    void print(const std::string& s = "Conditional", const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /** check equality */
    bool equals(const This& c, double tol = 1e-9) const;

    /// @}

    /// @name Standard Interface
    /// @{

    virtual ~Conditional() {}

    /** return the number of frontals */
    size_t nrFrontals() const { return nrFrontals_; }

    /** return the number of parents */
    size_t nrParents() const { return asFactor().size() - nrFrontals_; }

    /** Convenience function to get the first frontal key */
    Key firstFrontalKey() const {
      if(nrFrontals_ > 0)
        return asFactor().front();
      else
        throw std::invalid_argument("Requested Conditional::firstFrontalKey from a conditional with zero frontal keys");
    }

    /** return a view of the frontal keys */
    Frontals frontals() const { return ConstFactorRangeIterator({beginFrontals(), endFrontals()});}

    /** return a view of the parent keys */
    Parents parents() const { return ConstFactorRangeIterator({beginParents(), endParents()}); }

    /**
     * All conditional types need to implement a `logProbability` function, for which
     *   exp(logProbability(x)) = evaluate(x).
     */
    virtual double logProbability(const HybridValues& c) const;

    /**
     * All conditional types need to implement an `evaluate` function, that yields
     * a true probability. The default implementation just exponentiates logProbability.
     */
    virtual double evaluate(const HybridValues& c) const;

    /// Evaluate probability density, sugar.
    double operator()(const HybridValues& x) const {
      return evaluate(x);
    }

    /**
     * All conditional types need to implement a log normalization constant to
     * make it such that error>=0.
     */
    virtual double logNormalizationConstant() const;

    /** Non-virtual, exponentiate logNormalizationConstant. */
    double normalizationConstant() const;

    /// @}
    /// @name Advanced Interface
    /// @{

    /** Iterator pointing to first frontal key. */
    typename FACTOR::const_iterator beginFrontals() const { return asFactor().begin(); }

    /** Iterator pointing past the last frontal key. */
    typename FACTOR::const_iterator endFrontals() const { return asFactor().begin() + nrFrontals_; }

    /** Iterator pointing to the first parent key. */
    typename FACTOR::const_iterator beginParents() const { return endFrontals(); }

    /** Iterator pointing past the last parent key. */
    typename FACTOR::const_iterator endParents() const { return asFactor().end(); }

    /** Mutable version of nrFrontals */
    size_t& nrFrontals() { return nrFrontals_; }

    /** Mutable iterator pointing to first frontal key. */
    typename FACTOR::iterator beginFrontals() { return asFactor().begin(); }

    /** Mutable iterator pointing past the last frontal key. */
    typename FACTOR::iterator endFrontals() { return asFactor().begin() + nrFrontals_; }

    /** Mutable iterator pointing to the first parent key. */
    typename FACTOR::iterator beginParents() { return asFactor().begin() + nrFrontals_; }

    /** Mutable iterator pointing past the last parent key. */
    typename FACTOR::iterator endParents() { return asFactor().end(); }

    /**
     * Check invariants of this conditional, given the values `x`.
     * It tests:
     *  - evaluate >= 0.0
     *  - evaluate(x) == conditional(x)
     *  - exp(logProbability(x)) == evaluate(x)
     *  - logNormalizationConstant() = log(normalizationConstant())
     *  - error >= 0.0
     *  - logProbability(x) == logNormalizationConstant() - error(x)
     *
     * @param conditional The conditional to test, as a reference to the derived type.
     * @tparam VALUES HybridValues, or a more narrow type like DiscreteValues.
    */
    template <class VALUES>
    static bool CheckInvariants(const DERIVEDCONDITIONAL& conditional,
                                const VALUES& x);

    /// @}

  private:

    /// @name Serialization
    /// @{

    // Cast to factor type (non-const) (casts down to derived conditional type, then up to factor type)
    FACTOR& asFactor() { return static_cast<FACTOR&>(static_cast<DERIVEDCONDITIONAL&>(*this)); }

    // Cast to derived type (const) (casts down to derived conditional type, then up to factor type)
    const FACTOR& asFactor() const { return static_cast<const FACTOR&>(static_cast<const DERIVEDCONDITIONAL&>(*this)); }

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(nrFrontals_);
    }
#endif

    /// @}

  };

} // gtsam
