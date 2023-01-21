/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactorGraph.h
 * @brief   Factor Graph consisting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <cstddef>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/GraphvizFormatting.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <boost/shared_ptr.hpp>
#include <functional>

namespace gtsam {

  // Forward declarations
  class Values;
  class Ordering;
  class GaussianFactorGraph;
  class SymbolicFactorGraph;
  template<typename T>
  class Expression;
  template<typename T>
  class ExpressionFactor;

  /**
   * A NonlinearFactorGraph is a graph of non-Gaussian, i.e. non-linear factors,
   * which derive from NonlinearFactor. The values structures are typically (in
   * SAM) more general than just vectors, e.g., Rot3 or Pose3, which are objects
   * in non-linear manifolds. Linearizing the non-linear factor graph creates a
   * linear factor graph on the tangent vector space at the linearization point.
   * Because the tangent space is a true vector space, the config type will be
   * an VectorValues in that linearized factor graph.
   * @addtogroup nonlinear
   */
  class GTSAM_EXPORT NonlinearFactorGraph: public FactorGraph<NonlinearFactor> {

  public:

    typedef FactorGraph<NonlinearFactor> Base;
    typedef NonlinearFactorGraph This;
    typedef boost::shared_ptr<This> shared_ptr;

    /// @name Standard Constructors
    /// @{

    /** Default constructor */
    NonlinearFactorGraph() {}

    /** Construct from iterator over factors */
    template<typename ITERATOR>
    NonlinearFactorGraph(ITERATOR firstFactor, ITERATOR lastFactor) : Base(firstFactor, lastFactor) {}

    /** Construct from container of factors (shared_ptr or plain objects) */
    template<class CONTAINER>
    explicit NonlinearFactorGraph(const CONTAINER& factors) : Base(factors) {}

    /** Implicit copy/downcast constructor to override explicit template container constructor */
    template<class DERIVEDFACTOR>
    NonlinearFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph) : Base(graph) {}

    /// Destructor
    virtual ~NonlinearFactorGraph() {}

    /// @}
    /// @name Testable
    /// @{

    /** print */
    void print(
        const std::string& str = "NonlinearFactorGraph: ",
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

    /** print errors along with factors*/
    void printErrors(const Values& values, const std::string& str = "NonlinearFactorGraph: ",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter,
      const std::function<bool(const Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
        printCondition = [](const Factor *,double, size_t) {return true;}) const;

    /** Test equality */
    bool equals(const NonlinearFactorGraph& other, double tol = 1e-9) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /** unnormalized error, \f$ \sum_i 0.5 (h_i(X_i)-z)^2 / \sigma^2 \f$ in the most common case */
    double error(const Values& values) const;

    /** Unnormalized probability. O(n) */
    double probPrime(const Values& values) const;

    /**
     * Create a symbolic factor graph
     */
    boost::shared_ptr<SymbolicFactorGraph> symbolic() const;

    /**
     * Compute a fill-reducing ordering using COLAMD.
     */
    Ordering orderingCOLAMD() const;

    /**
     * Compute a fill-reducing ordering with constraints using CCOLAMD
     *
     * @param constraints is a map of Key->group, where 0 is unconstrained, and higher
     * group numbers are further back in the ordering. Only keys with nonzero group
     * indices need to appear in the constraints, unconstrained is assumed for all
     * other variables
     */
    Ordering orderingCOLAMDConstrained(const FastMap<Key, int>& constraints) const;

    /// Linearize a nonlinear factor graph
    boost::shared_ptr<GaussianFactorGraph> linearize(const Values& linearizationPoint) const;

    /// typdef for dampen functions used below
    typedef std::function<void(const boost::shared_ptr<HessianFactor>& hessianFactor)> Dampen;

    /**
     * Instead of producing a GaussianFactorGraph, pre-allocate and linearize directly
     * into a HessianFactor. Avoids the many mallocs and pointer indirection in constructing
     * a new graph, and hence useful in case a dense solve is appropriate for your problem.
     * An optional lambda function can be used to apply damping on the filled Hessian.
     * No parallelism is exploited, because all the factors write in the same memory.
     */
    boost::shared_ptr<HessianFactor> linearizeToHessianFactor(
        const Values& values, const Dampen& dampen = nullptr) const;

    /**
     * Instead of producing a GaussianFactorGraph, pre-allocate and linearize directly
     * into a HessianFactor. Avoids the many mallocs and pointer indirection in constructing
     * a new graph, and hence useful in case a dense solve is appropriate for your problem.
     * An ordering is given that still decides how the Hessian is laid out.
     * An optional lambda function can be used to apply damping on the filled Hessian.
     * No parallelism is exploited, because all the factors write in the same memory.
     */
    boost::shared_ptr<HessianFactor> linearizeToHessianFactor(
        const Values& values, const Ordering& ordering, const Dampen& dampen = nullptr) const;

    /// Linearize and solve in one pass.
    /// Calls linearizeToHessianFactor, densely solves the normal equations, and updates the values.
    Values updateCholesky(const Values& values,
                          const Dampen& dampen = nullptr) const;

    /// Linearize and solve in one pass.
    /// Calls linearizeToHessianFactor, densely solves the normal equations, and updates the values.
    Values updateCholesky(const Values& values, const Ordering& ordering,
                          const Dampen& dampen = nullptr) const;

    /// Clone() performs a deep-copy of the graph, including all of the factors
    NonlinearFactorGraph clone() const;

    /**
     * Rekey() performs a deep-copy of all of the factors, and changes
     * keys according to a mapping.
     *
     * Keys not specified in the mapping will remain unchanged.
     *
     * @param rekey_mapping is a map of old->new keys
     * @result a cloned graph with updated keys
     */
    NonlinearFactorGraph rekey(const std::map<Key,Key>& rekey_mapping) const;

    /**
     * Directly add ExpressionFactor that implements |h(x)-z|^2_R
     * @param h expression that implements measurement function
     * @param z measurement
     * @param R model
     */
    template<typename T>
    void addExpressionFactor(const SharedNoiseModel& R, const T& z,
                             const Expression<T>& h) {
      push_back(boost::make_shared<ExpressionFactor<T> >(R, z, h));
    }

    /**
     * Convenience method which adds a PriorFactor to the factor graph.
     * @param key    Variable key
     * @param prior  The variable's prior value
     * @param model  Noise model for prior factor
     */
    template<typename T>
    void addPrior(Key key, const T& prior,
                  const SharedNoiseModel& model = nullptr) {
      emplace_shared<PriorFactor<T>>(key, prior, model);
    }

    /**
     * Convenience method which adds a PriorFactor to the factor graph.
     * @param key         Variable key
     * @param prior       The variable's prior value
     * @param covariance  Covariance matrix.
     * 
     * Note that the smart noise model associated with the prior factor
     * automatically picks the right noise model (e.g. a diagonal noise model
     * if the provided covariance matrix is diagonal).
     */
    template<typename T>
    void addPrior(Key key, const T& prior, const Matrix& covariance) {
      emplace_shared<PriorFactor<T>>(key, prior, covariance);
    }

    /// @}
    /// @name Graph Display
    /// @{

    using FactorGraph::dot;
    using FactorGraph::saveGraph;

    /// Output to graphviz format, stream version, with Values/extra options.
    void dot(std::ostream& os, const Values& values,
             const KeyFormatter& keyFormatter = DefaultKeyFormatter,
             const GraphvizFormatting& writer = GraphvizFormatting()) const;

    /// Output to graphviz format string, with Values/extra options.
    std::string dot(
        const Values& values,
        const KeyFormatter& keyFormatter = DefaultKeyFormatter,
        const GraphvizFormatting& writer = GraphvizFormatting()) const;

    /// output to file with graphviz format, with Values/extra options.
    void saveGraph(
        const std::string& filename, const Values& values,
        const KeyFormatter& keyFormatter = DefaultKeyFormatter,
        const GraphvizFormatting& writer = GraphvizFormatting()) const;
    /// @}

   private:

    /**
     * Linearize from Scatter rather than from Ordering.  Made private because
     *  it doesn't include gttic.
     */
    boost::shared_ptr<HessianFactor> linearizeToHessianFactor(
        const Values& values, const Scatter& scatter, const Dampen& dampen = nullptr) const;

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NonlinearFactorGraph",
                boost::serialization::base_object<Base>(*this));
    }
  };

/// traits
template<>
struct traits<NonlinearFactorGraph> : public Testable<NonlinearFactorGraph> {
};

} //\ namespace gtsam

