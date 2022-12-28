/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianConditional.h
 * @brief   Conditional Gaussian Base class
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <boost/utility.hpp>

#include <gtsam/global_includes.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/linear/VectorValues.h>

#include <random> // for std::mt19937_64 

namespace gtsam {

  /**
  * A GaussianConditional functions as the node in a Bayes network.
  * It has a set of parents y,z, etc. and implements a probability density on x.
  * The negative log-probability is given by \f$ \frac{1}{2} |Rx - (d - Sy - Tz - ...)|^2 \f$
  * @ingroup linear
  */
  class GTSAM_EXPORT GaussianConditional :
    public JacobianFactor,
    public Conditional<JacobianFactor, GaussianConditional>
  {
  public:
    typedef GaussianConditional This; ///< Typedef to this class
    typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class
    typedef JacobianFactor BaseFactor; ///< Typedef to our factor base class
    typedef Conditional<BaseFactor, This> BaseConditional; ///< Typedef to our conditional base class

    /// @name Constructors
    /// @{

    /** default constructor needed for serialization */
    GaussianConditional() {}

    /** constructor with no parents |Rx-d| */
    GaussianConditional(Key key, const Vector& d, const Matrix& R,
      const SharedDiagonal& sigmas = SharedDiagonal());

    /** constructor with only one parent |Rx+Sy-d| */
    GaussianConditional(Key key, const Vector& d, const Matrix& R, Key parent1,
                        const Matrix& S,
                        const SharedDiagonal& sigmas = SharedDiagonal());

    /** constructor with two parents |Rx+Sy+Tz-d| */
    GaussianConditional(Key key, const Vector& d, const Matrix& R, Key parent1,
                        const Matrix& S, Key parent2, const Matrix& T,
                        const SharedDiagonal& sigmas = SharedDiagonal());

    /** Constructor with arbitrary number of frontals and parents.
    *   @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
    *           collection of keys and matrices making up the conditional. */
    template<typename TERMS>
    GaussianConditional(const TERMS& terms,
      size_t nrFrontals, const Vector& d,
      const SharedDiagonal& sigmas = SharedDiagonal());

    /** Constructor with arbitrary number keys, and where the augmented matrix is given all together
     *  instead of in block terms.  Note that only the active view of the provided augmented matrix
     *  is used, and that the matrix data is copied into a newly-allocated matrix in the constructed
     *  factor. */
    template<typename KEYS>
    GaussianConditional(
      const KEYS& keys, size_t nrFrontals, const VerticalBlockMatrix& augmentedMatrix,
      const SharedDiagonal& sigmas = SharedDiagonal());

    /// Construct from mean A1 p1 + b and standard deviation.
    static GaussianConditional FromMeanAndStddev(Key key, const Matrix& A,
                                                 Key parent, const Vector& b,
                                                 double sigma);

    /// Construct from mean A1 p1 + A2 p2 + b and standard deviation.
    static GaussianConditional FromMeanAndStddev(Key key,  //
                                                 const Matrix& A1, Key parent1,
                                                 const Matrix& A2, Key parent2,
                                                 const Vector& b, double sigma);

    /** Combine several GaussianConditional into a single dense GC.  The conditionals enumerated by
    *   \c first and \c last must be in increasing order, meaning that the parents of any
    *   conditional may not include a conditional coming before it.
    *   @param firstConditional Iterator to the first conditional to combine, must dereference to a
    *          shared_ptr<GaussianConditional>.
    *   @param lastConditional Iterator to after the last conditional to combine, must dereference
    *          to a shared_ptr<GaussianConditional>. */
    template<typename ITERATOR>
    static shared_ptr Combine(ITERATOR firstConditional, ITERATOR lastConditional);

    /// @}
    /// @name Testable
    /// @{

    /** print */
    void print(
        const std::string& = "GaussianConditional",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override;

    /** equals function */
    bool equals(const GaussianFactor&cg, double tol = 1e-9) const override;

    /// @}
    /// @name Standard Interface
    /// @{

    /**
     * Calculate log-density for given values `x`:
     *   -0.5*(error + n*log(2*pi) + log det(Sigma))
     * where x is the vector of values, and Sigma is the covariance matrix.
     */
    double logDensity(const VectorValues& x) const;

    /**
     * Calculate probability density for given values `x`:
     *   exp(-0.5*error(x)) / sqrt((2*pi)^n*det(Sigma))
     * where x is the vector of values, and Sigma is the covariance matrix.
     */
    double evaluate(const VectorValues& x) const;

    /** Return a view of the upper-triangular R block of the conditional */
    constABlock R() const { return Ab_.range(0, nrFrontals()); }

    /** Get a view of the parent blocks. */
    constABlock S() const { return Ab_.range(nrFrontals(), size()); }

    /** Get a view of the S matrix for the variable pointed to by the given key iterator */
    constABlock S(const_iterator it) const { return BaseFactor::getA(it); }

    /** Get a view of the r.h.s. vector d */
    const constBVector d() const { return BaseFactor::getb(); }

    /**
     * @brief Compute the log determinant of the R matrix.
     * For numerical stability, the determinant is computed in log
     * form, so it is a summation rather than a multiplication.
     *
     * @return double
     */
    double logDeterminant() const;

    /**
     * @brief Compute the determinant of the R matrix.
     *
     * The determinant is computed in log form (hence summation) for numerical
     * stability and then exponentiated.
     *
     * @return double
     */
    double determinant() const { return exp(this->logDeterminant()); }

    /**
    * Solves a conditional Gaussian and writes the solution into the entries of
    * \c x for each frontal variable of the conditional.  The parents are
    * assumed to have already been solved in and their values are read from \c x.
    * This function works for multiple frontal variables.
    *
    * Given the Gaussian conditional with log likelihood \f$ |R x_f - (d - S x_s)|^2 \f$,
    * where \f$ f \f$ are the frontal variables and \f$ s \f$ are the separator
    * variables of this conditional, this solve function computes
    * \f$ x_f = R^{-1} (d - S x_s) \f$ using back-substitution.
    *
    * @param parents VectorValues containing solved parents \f$ x_s \f$.
    */
    VectorValues solve(const VectorValues& parents) const;

    VectorValues solveOtherRHS(const VectorValues& parents, const VectorValues& rhs) const;

    /** Performs transpose backsubstition in place on values */
    void solveTransposeInPlace(VectorValues& gy) const;

    /** Convert to a likelihood factor by providing value before bar. */
    JacobianFactor::shared_ptr likelihood(
        const VectorValues& frontalValues) const;

    /** Single variable version of likelihood. */
    JacobianFactor::shared_ptr likelihood(const Vector& frontal) const;

    /**
     * Sample from conditional, zero parent version
     * Example:
     *   std::mt19937_64 rng(42);
     *   auto sample = gbn.sample(&rng);
     */
    VectorValues sample(std::mt19937_64* rng) const;

    /**
     * Sample from conditional, given missing variables
     * Example:
     *   std::mt19937_64 rng(42);
     *   VectorValues given = ...;
     *   auto sample = gbn.sample(given, &rng);
     */
    VectorValues sample(const VectorValues& parentsValues,
                        std::mt19937_64* rng) const;

    /// Sample, use default rng
    VectorValues sample() const;

    /// Sample with given values, use default rng
    VectorValues sample(const VectorValues& parentsValues) const;

    /// @}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
    /// @name Deprecated
    /// @{
    /** Scale the values in \c gy according to the sigmas for the frontal variables in this
     *  conditional. */
    void GTSAM_DEPRECATED scaleFrontalsBySigma(VectorValues& gy) const;
    /// @}
#endif

   private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseFactor);
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
    }
  }; // GaussianConditional

/// traits
template<>
struct traits<GaussianConditional> : public Testable<GaussianConditional> {};

} // \ namespace gtsam

#include <gtsam/linear/GaussianConditional-inl.h>

