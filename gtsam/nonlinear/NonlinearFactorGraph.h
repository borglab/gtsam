/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactorGraph.h
 * @brief   Factor Graph Constsiting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/FactorGraph.h>

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
   * Formatting options when saving in GraphViz format using
   * NonlinearFactorGraph::saveGraph.
   */
  struct GTSAM_EXPORT GraphvizFormatting {
    enum Axis { X, Y, Z, NEGX, NEGY, NEGZ }; ///< World axes to be assigned to paper axes
    Axis paperHorizontalAxis; ///< The world axis assigned to the horizontal paper axis
    Axis paperVerticalAxis; ///< The world axis assigned to the vertical paper axis
    double figureWidthInches; ///< The figure width on paper in inches
    double figureHeightInches; ///< The figure height on paper in inches
    double scale; ///< Scale all positions to reduce / increase density
    bool mergeSimilarFactors; ///< Merge multiple factors that have the same connectivity
    bool plotFactorPoints; ///< Plots each factor as a dot between the variables
    bool connectKeysToFactor; ///< Draw a line from each key within a factor to the dot of the factor
    bool binaryEdges; ///< just use non-dotted edges for binary factors
    std::map<size_t, Point2> factorPositions; ///< (optional for each factor) Manually specify factor "dot" positions.
    /// Default constructor sets up robot coordinates.  Paper horizontal is robot Y,
    /// paper vertical is robot X.  Default figure size of 5x5 in.
    GraphvizFormatting() :
      paperHorizontalAxis(Y), paperVerticalAxis(X),
      figureWidthInches(5), figureHeightInches(5), scale(1),
      mergeSimilarFactors(false), plotFactorPoints(true),
      connectKeysToFactor(true), binaryEdges(true) {}
  };


  /**
   * A non-linear factor graph is a graph of non-Gaussian, i.e. non-linear factors,
   * which derive from NonlinearFactor. The values structures are typically (in SAM) more general
   * than just vectors, e.g., Rot3 or Pose3, which are objects in non-linear manifolds.
   * Linearizing the non-linear factor graph creates a linear factor graph on the
   * tangent vector space at the linearization point. Because the tangent space is a true
   * vector space, the config type will be an VectorValues in that linearized factor graph.
   */
  class GTSAM_EXPORT NonlinearFactorGraph: public FactorGraph<NonlinearFactor> {

  public:

    typedef FactorGraph<NonlinearFactor> Base;
    typedef NonlinearFactorGraph This;
    typedef boost::shared_ptr<This> shared_ptr;

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

    /** print */
    void print(const std::string& str = "NonlinearFactorGraph: ",
               const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /** print errors along with factors*/
    void printErrors(const Values& values, const std::string& str = "NonlinearFactorGraph: ",
                     const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /** Test equality */
    bool equals(const NonlinearFactorGraph& other, double tol = 1e-9) const;

    /** Write the graph in GraphViz format for visualization */
    void saveGraph(std::ostream& stm, const Values& values = Values(),
      const GraphvizFormatting& graphvizFormatting = GraphvizFormatting(),
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /** unnormalized error, \f$ 0.5 \sum_i (h_i(X_i)-z)^2/\sigma^2 \f$ in the most common case */
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

    //[MH-A]: for both NMF and MHNMF
    boost::shared_ptr<GaussianFactorGraph> mhLinearize(const Values& linearizationPoint) const;
    
    /// typdef for dampen functions used below
    typedef std::function<void(const boost::shared_ptr<HessianFactor>& hessianFactor)> Dampen;

    /**
     * Instead of producing a GaussianFactorGraph, pre-allocate and linearize directly
     * into a HessianFactor. Avoids the many mallocs and pointer indirection in constructing
     * a new graph, and hence useful in case a dense solve is appropriate for your problem.
     * An optional ordering can be given that still decides how the Hessian is laid out.
     * An optional lambda function can be used to apply damping on the filled Hessian.
     * No parallelism is exploited, because all the factors write in the same memory.
     */
    boost::shared_ptr<HessianFactor> linearizeToHessianFactor(
        const Values& values, boost::optional<Ordering&> ordering = boost::none,
        const Dampen& dampen = nullptr) const;

    /// Linearize and solve in one pass.
    /// Calls linearizeToHessianFactor, densely solves the normal equations, and updates the values.
    Values updateCholesky(const Values& values, boost::optional<Ordering&> ordering = boost::none,
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

  private:

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

