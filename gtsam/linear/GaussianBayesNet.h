/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   GaussianBayesNet
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/global_includes.h>

#include <utility>
namespace gtsam {

  /** 
   * GaussianBayesNet is a Bayes net made from linear-Gaussian conditionals.
   * @ingroup linear
   */
  class GTSAM_EXPORT GaussianBayesNet: public BayesNet<GaussianConditional>
  {
  public:

    typedef BayesNet<GaussianConditional> Base;
    typedef GaussianBayesNet This;
    typedef GaussianConditional ConditionalType;
    typedef std::shared_ptr<This> shared_ptr;
    typedef std::shared_ptr<ConditionalType> sharedConditional;

    /// @name Standard Constructors
    /// @{

    /** Construct empty bayes net */
    GaussianBayesNet() {}

    /** Construct from iterator over conditionals */
    template <typename ITERATOR>
    GaussianBayesNet(ITERATOR firstConditional, ITERATOR lastConditional)
        : Base(firstConditional, lastConditional) {}

    /** Construct from container of factors (shared_ptr or plain objects) */
    template <class CONTAINER>
    explicit GaussianBayesNet(const CONTAINER& conditionals) {
      push_back(conditionals);
    }

    /** Implicit copy/downcast constructor to override explicit template
     * container constructor */
    template <class DERIVEDCONDITIONAL>
    explicit GaussianBayesNet(const FactorGraph<DERIVEDCONDITIONAL>& graph)
        : Base(graph) {}

    /**
     * Constructor that takes an initializer list of shared pointers.
     *  BayesNet bn = {make_shared<Conditional>(), ...};
     */
    template <class DERIVEDCONDITIONAL>
    GaussianBayesNet(
        std::initializer_list<std::shared_ptr<DERIVEDCONDITIONAL> > conditionals)
        : Base(conditionals) {}

    /// Destructor
    virtual ~GaussianBayesNet() = default;

    /// @}

    /// @name Testable
    /// @{

    /** Check equality */
    bool equals(const This& bn, double tol = 1e-9) const;

    /// print graph
    void print(
        const std::string& s = "",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override {
      Base::print(s, formatter);
    }

    /// @}

    /// @name Standard Interface
    /// @{

    /// Sum error over all variables.
    double error(const VectorValues& x) const;

    /// Sum logProbability over all variables.
    double logProbability(const VectorValues& x) const;

    /**
     * Calculate probability density for given values `x`:
     *   exp(logProbability)
     * where x is the vector of values.
     */
    double evaluate(const VectorValues& x) const;

    /// Evaluate probability density, sugar.
    double operator()(const VectorValues& x) const {
      return evaluate(x);
    }

    /// Solve the GaussianBayesNet, i.e. return \f$ x = R^{-1}*d \f$, by
    /// back-substitution
    VectorValues optimize() const;

    /// Version of optimize for incomplete BayesNet, given missing variables
    VectorValues optimize(const VectorValues& given) const;

    /**
     * Sample using ancestral sampling
     * Example:
     *   std::mt19937_64 rng(42);
     *   auto sample = gbn.sample(&rng);
     */
    VectorValues sample(std::mt19937_64* rng) const;

    /**
     * Sample from an incomplete BayesNet, given missing variables
     * Example:
     *   std::mt19937_64 rng(42);
     *   VectorValues given = ...;
     *   auto sample = gbn.sample(given, &rng);
     */
    VectorValues sample(const VectorValues& given, std::mt19937_64* rng) const;

    /// Sample using ancestral sampling, use default rng
    VectorValues sample() const;

    /// Sample from an incomplete BayesNet, use default rng
    VectorValues sample(const VectorValues& given) const;

    /**
     * Return ordering corresponding to a topological sort.
     * There are many topological sorts of a Bayes net. This one
     * corresponds to the one that makes 'matrix' below upper-triangular.
     * In case Bayes net is incomplete any non-frontal are added to the end.
     */
    Ordering ordering() const;

    ///@}

    ///@name Linear Algebra
    ///@{

    /**
     * Return (dense) upper-triangular matrix representation
     * Will return upper-triangular matrix only when using 'ordering' above.
     * In case Bayes net is incomplete zero columns are added to the end.
     */
    std::pair<Matrix, Vector> matrix(const Ordering& ordering) const;

    /**
     * Return (dense) upper-triangular matrix representation
     * Will return upper-triangular matrix only when using 'ordering' above.
     * In case Bayes net is incomplete zero columns are added to the end.
     */
    std::pair<Matrix, Vector> matrix() const;

    /**
     * Optimize along the gradient direction, with a closed-form computation to perform the line
     * search.  The gradient is computed about \f$ \delta x=0 \f$.
     *
     * This function returns \f$ \delta x \f$ that minimizes a reparametrized problem.  The error
     * function of a GaussianBayesNet is
     *
     * \f[ f(\delta x) = \frac{1}{2} |R \delta x - d|^2 = \frac{1}{2}d^T d - d^T R \delta x +
     * \frac{1}{2} \delta x^T R^T R \delta x \f]
     *
     * with gradient and Hessian
     *
     * \f[ g(\delta x) = R^T(R\delta x - d), \qquad G(\delta x) = R^T R. \f]
     *
     * This function performs the line search in the direction of the gradient evaluated at \f$ g =
     * g(\delta x = 0) \f$ with step size \f$ \alpha \f$ that minimizes \f$ f(\delta x = \alpha g)
     * \f$:
     *
     * \f[ f(\alpha) = \frac{1}{2} d^T d + g^T \delta x + \frac{1}{2} \alpha^2 g^T G g \f]
     *
     * Optimizing by setting the derivative to zero yields \f$ \hat \alpha = (-g^T g) / (g^T G g)
     * \f$.  For efficiency, this function evaluates the denominator without computing the Hessian
     * \f$ G \f$, returning
     *
     * \f[ \delta x = \hat\alpha g = \frac{-g^T g}{(R g)^T(R g)} \f] */
    VectorValues optimizeGradientSearch() const;

    /** Compute the gradient of the energy function, \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} R x -
     * d \right\Vert^2 \f$, centered around \f$ x = x_0 \f$. The gradient is \f$ R^T(Rx-d) \f$.
     *
     * @param x0 The center about which to compute the gradient
     * @return The gradient as a VectorValues */
    VectorValues gradient(const VectorValues& x0) const;

    /** Compute the gradient of the energy function, \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} R x - d
     * \right\Vert^2 \f$, centered around zero. The gradient about zero is \f$ -R^T d \f$.  See also
     * gradient(const GaussianBayesNet&, const VectorValues&).
     *
     * @param [output] g A VectorValues to store the gradient, which must be preallocated, see
     *        allocateVectorValues */
    VectorValues gradientAtZero() const;

    /**
     * Computes the determinant of a GassianBayesNet. A GaussianBayesNet is an upper triangular
     * matrix and for an upper triangular matrix determinant is the product of the diagonal
     * elements. Instead of actually multiplying we add the logarithms of the diagonal elements and
     * take the exponent at the end because this is more numerically stable.
     * @param bayesNet The input GaussianBayesNet
     * @return The determinant */
    double determinant() const;

    /**
     * Computes the log of the determinant of a GassianBayesNet. A GaussianBayesNet is an upper
     * triangular matrix and for an upper triangular matrix determinant is the product of the
     * diagonal elements.
     * @param bayesNet The input GaussianBayesNet
     * @return The determinant */
    double logDeterminant() const;

    /**
     * Backsubstitute with a different RHS vector than the one stored in this BayesNet.
     * gy=inv(R*inv(Sigma))*gx
     */
    VectorValues backSubstitute(const VectorValues& gx) const;

    /**
     * Transpose backsubstitute with a different RHS vector than the one stored in this BayesNet.
     * gy=inv(L)*gx by solving L*gy=gx.
     * gy=inv(R'*inv(Sigma))*gx
     * gz'*R'=gx', gy = gz.*sigmas
     */
    VectorValues backSubstituteTranspose(const VectorValues& gx) const;

    /// @}
    /// @name HybridValues methods.
    /// @{

    using Base::evaluate; // Expose evaluate(const HybridValues&) method..
    using Base::logProbability; // Expose logProbability(const HybridValues&) method..
    using Base::error; // Expose error(const HybridValues&) method..

    /// @}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
  };

  /// traits
  template<>
  struct traits<GaussianBayesNet> : public Testable<GaussianBayesNet> {
  };

} //\ namespace gtsam
