/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess, Richard Roberts
 */

// \callgraph

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>
#include <gtsam/linear/GaussianBayesTree.h>

#include <boost/variant.hpp>

namespace gtsam {

/**
 * @addtogroup ISAM2
 * Parameters for ISAM2 using Gauss-Newton optimization.  Either this class or
 * ISAM2DoglegParams should be specified as the optimizationParams in
 * ISAM2Params, which should in turn be passed to ISAM2(const ISAM2Params&).
 */
struct GTSAM_EXPORT ISAM2GaussNewtonParams {
  double wildfireThreshold; ///< Continue updating the linear delta only when changes are above this threshold (default: 0.001)

  /** Specify parameters as constructor arguments */
  ISAM2GaussNewtonParams(
      double _wildfireThreshold = 0.001 ///< see ISAM2GaussNewtonParams public variables, ISAM2GaussNewtonParams::wildfireThreshold
  ) : wildfireThreshold(_wildfireThreshold) {}

  void print(const std::string str = "") const {
    std::cout << str << "type:              ISAM2GaussNewtonParams\n";
    std::cout << str << "wildfireThreshold: " << wildfireThreshold << "\n";
    std::cout.flush();
  }

  double getWildfireThreshold() const { return wildfireThreshold; }
  void setWildfireThreshold(double wildfireThreshold) { this->wildfireThreshold = wildfireThreshold; }
};

/**
 * @addtogroup ISAM2
 * Parameters for ISAM2 using Dogleg optimization.  Either this class or
 * ISAM2GaussNewtonParams should be specified as the optimizationParams in
 * ISAM2Params, which should in turn be passed to ISAM2(const ISAM2Params&).
 */
struct GTSAM_EXPORT ISAM2DoglegParams {
  double initialDelta; ///< The initial trust region radius for Dogleg
  double wildfireThreshold; ///< Continue updating the linear delta only when changes are above this threshold (default: 1e-5)
  DoglegOptimizerImpl::TrustRegionAdaptationMode adaptationMode; ///< See description in DoglegOptimizerImpl::TrustRegionAdaptationMode
  bool verbose; ///< Whether Dogleg prints iteration and convergence information

  /** Specify parameters as constructor arguments */
  ISAM2DoglegParams(
      double _initialDelta = 1.0, ///< see ISAM2DoglegParams::initialDelta
      double _wildfireThreshold = 1e-5, ///< see ISAM2DoglegParams::wildfireThreshold
      DoglegOptimizerImpl::TrustRegionAdaptationMode _adaptationMode = DoglegOptimizerImpl::SEARCH_EACH_ITERATION, ///< see ISAM2DoglegParams::adaptationMode
      bool _verbose = false ///< see ISAM2DoglegParams::verbose
  ) : initialDelta(_initialDelta), wildfireThreshold(_wildfireThreshold),
  adaptationMode(_adaptationMode), verbose(_verbose) {}

  void print(const std::string str = "") const {
    std::cout << str << "type:              ISAM2DoglegParams\n";
    std::cout << str << "initialDelta:      " << initialDelta << "\n";
    std::cout << str << "wildfireThreshold: " << wildfireThreshold << "\n";
    std::cout << str << "adaptationMode:    " << adaptationModeTranslator(adaptationMode) << "\n";
    std::cout.flush();
  }

  double getInitialDelta() const { return initialDelta; }
  double getWildfireThreshold() const { return wildfireThreshold; }
  std::string getAdaptationMode() const { return adaptationModeTranslator(adaptationMode); };
  bool isVerbose() const { return verbose; };

  void setInitialDelta(double initialDelta) { this->initialDelta = initialDelta; }
  void setWildfireThreshold(double wildfireThreshold) { this->wildfireThreshold = wildfireThreshold; }
  void setAdaptationMode(const std::string& adaptationMode) { this->adaptationMode = adaptationModeTranslator(adaptationMode); }
  void setVerbose(bool verbose) { this->verbose = verbose; };

  std::string adaptationModeTranslator(const DoglegOptimizerImpl::TrustRegionAdaptationMode& adaptationMode) const;
  DoglegOptimizerImpl::TrustRegionAdaptationMode adaptationModeTranslator(const std::string& adaptationMode) const;
};

/**
 * @addtogroup ISAM2
 * Parameters for the ISAM2 algorithm.  Default parameter values are listed below.
 */
typedef FastMap<char,Vector> ISAM2ThresholdMap;
typedef ISAM2ThresholdMap::value_type ISAM2ThresholdMapValue;
struct GTSAM_EXPORT ISAM2Params {
  typedef boost::variant<ISAM2GaussNewtonParams, ISAM2DoglegParams> OptimizationParams; ///< Either ISAM2GaussNewtonParams or ISAM2DoglegParams
  typedef boost::variant<double, FastMap<char,Vector> > RelinearizationThreshold; ///< Either a constant relinearization threshold or a per-variable-type set of thresholds

  /** Optimization parameters, this both selects the nonlinear optimization
   * method and specifies its parameters, either ISAM2GaussNewtonParams or
   * ISAM2DoglegParams.  In the former, Gauss-Newton optimization will be used
   * with the specified parameters, and in the latter Powell's dog-leg
   * algorithm will be used with the specified parameters.
   */
  OptimizationParams optimizationParams;

  /** Only relinearize variables whose linear delta magnitude is greater than
   * this threshold (default: 0.1).  If this is a FastMap<char,Vector> instead
   * of a double, then the threshold is specified for each dimension of each
   * variable type.  This parameter then maps from a character indicating the
   * variable type to a Vector of thresholds for each dimension of that
   * variable.  For example, if Pose keys are of type TypedSymbol<'x',Pose3>,
   * and landmark keys are of type TypedSymbol<'l',Point3>, then appropriate
   * entries would be added with:
   * \code
     FastMap<char,Vector> thresholds;
     thresholds['x'] = (Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished(); // 0.1 rad rotation threshold, 0.5 m translation threshold
     thresholds['l'] = Vector3(1.0, 1.0, 1.0);                // 1.0 m landmark position threshold
     params.relinearizeThreshold = thresholds;
     \endcode
   */
  RelinearizationThreshold relinearizeThreshold;

  int relinearizeSkip; ///< Only relinearize any variables every relinearizeSkip calls to ISAM2::update (default: 10)

  bool enableRelinearization; ///< Controls whether ISAM2 will ever relinearize any variables (default: true)

  bool evaluateNonlinearError; ///< Whether to evaluate the nonlinear error before and after the update, to return in ISAM2Result from update()

  enum Factorization { CHOLESKY, QR };
  /** Specifies whether to use QR or CHOESKY numerical factorization (default: CHOLESKY).
   * Cholesky is faster but potentially numerically unstable for poorly-conditioned problems, which can occur when
   * uncertainty is very low in some variables (or dimensions of variables) and very high in others.  QR is
   * slower but more numerically stable in poorly-conditioned problems.  We suggest using the default of Cholesky
   * unless gtsam sometimes throws IndefiniteLinearSystemException when your problem's Hessian is actually positive
   * definite.  For positive definite problems, numerical error accumulation can cause the problem to become
   * numerically negative or indefinite as solving proceeds, especially when using Cholesky.
   */
  Factorization factorization;

  /** Whether to cache linear factors (default: true).
   * This can improve performance if linearization is expensive, but can hurt
   * performance if linearization is very cleap due to computation to look up
   * additional keys.
   */
  bool cacheLinearizedFactors;

  KeyFormatter keyFormatter; ///< A KeyFormatter for when keys are printed during debugging (default: DefaultKeyFormatter)

  bool enableDetailedResults; ///< Whether to compute and return ISAM2Result::detailedResults, this can increase running time (default: false)

  /** Check variables for relinearization in tree-order, stopping the check once a variable does not need to be relinearized (default: false).
   * This can improve speed by only checking a small part of the top of the tree. However, variables below the check cut-off can accumulate
   * significant deltas without triggering relinearization. This is particularly useful in exploration scenarios where real-time performance
   * is desired over correctness. Use with caution.
   */
  bool enablePartialRelinearizationCheck;

  /// When you will be removing many factors, e.g. when using ISAM2 as a fixed-lag smoother, enable this option to
  /// add factors in the first available factor slots, to avoid accumulating NULL factor slots, at the cost of
  /// having to search for slots every time a factor is added.
  bool findUnusedFactorSlots;

  /** Specify parameters as constructor arguments */
  ISAM2Params(
      OptimizationParams _optimizationParams = ISAM2GaussNewtonParams(), ///< see ISAM2Params::optimizationParams
      RelinearizationThreshold _relinearizeThreshold = 0.1, ///< see ISAM2Params::relinearizeThreshold
      int _relinearizeSkip = 10, ///< see ISAM2Params::relinearizeSkip
      bool _enableRelinearization = true, ///< see ISAM2Params::enableRelinearization
      bool _evaluateNonlinearError = false, ///< see ISAM2Params::evaluateNonlinearError
      Factorization _factorization = ISAM2Params::CHOLESKY, ///< see ISAM2Params::factorization
      bool _cacheLinearizedFactors = true, ///< see ISAM2Params::cacheLinearizedFactors
      const KeyFormatter& _keyFormatter = DefaultKeyFormatter ///< see ISAM2::Params::keyFormatter
  ) : optimizationParams(_optimizationParams), relinearizeThreshold(_relinearizeThreshold),
      relinearizeSkip(_relinearizeSkip), enableRelinearization(_enableRelinearization),
      evaluateNonlinearError(_evaluateNonlinearError), factorization(_factorization),
      cacheLinearizedFactors(_cacheLinearizedFactors), keyFormatter(_keyFormatter),
      enableDetailedResults(false), enablePartialRelinearizationCheck(false),
      findUnusedFactorSlots(false) {}

  /// print iSAM2 parameters
  void print(const std::string& str = "") const {
    std::cout << str << "\n";
    if(optimizationParams.type() == typeid(ISAM2GaussNewtonParams))
      boost::get<ISAM2GaussNewtonParams>(optimizationParams).print("optimizationParams:                ");
    else if(optimizationParams.type() == typeid(ISAM2DoglegParams))
      boost::get<ISAM2DoglegParams>(optimizationParams).print("optimizationParams:                ");
    else
      std::cout << "optimizationParams:                " << "{unknown type}" << "\n";
    if(relinearizeThreshold.type() == typeid(double))
      std::cout << "relinearizeThreshold:              " << boost::get<double>(relinearizeThreshold) << "\n";
    else
    {
      std::cout << "relinearizeThreshold:              " << "{mapped}" << "\n";
      for(const ISAM2ThresholdMapValue& value: boost::get<ISAM2ThresholdMap>(relinearizeThreshold)) {
        std::cout << "                                   '" << value.first << "' -> [" << value.second.transpose() << " ]\n";
      }
    }
    std::cout << "relinearizeSkip:                   " << relinearizeSkip << "\n";
    std::cout << "enableRelinearization:             " << enableRelinearization << "\n";
    std::cout << "evaluateNonlinearError:            " << evaluateNonlinearError << "\n";
    std::cout << "factorization:                     " << factorizationTranslator(factorization) << "\n";
    std::cout << "cacheLinearizedFactors:            " << cacheLinearizedFactors << "\n";
    std::cout << "enableDetailedResults:             " << enableDetailedResults << "\n";
    std::cout << "enablePartialRelinearizationCheck: " << enablePartialRelinearizationCheck << "\n";
    std::cout << "findUnusedFactorSlots:             " << findUnusedFactorSlots << "\n";
    std::cout.flush();
  }

  /// @name Getters and Setters for all properties
  /// @{

  OptimizationParams getOptimizationParams() const { return this->optimizationParams; }
  RelinearizationThreshold getRelinearizeThreshold() const { return relinearizeThreshold; }
  int getRelinearizeSkip() const { return relinearizeSkip; }
  bool isEnableRelinearization() const { return enableRelinearization; }
  bool isEvaluateNonlinearError() const { return evaluateNonlinearError; }
  std::string getFactorization() const { return factorizationTranslator(factorization); }
  bool isCacheLinearizedFactors() const { return cacheLinearizedFactors; }
  KeyFormatter getKeyFormatter() const { return keyFormatter; }
  bool isEnableDetailedResults() const { return enableDetailedResults; }
  bool isEnablePartialRelinearizationCheck() const { return enablePartialRelinearizationCheck; }

  void setOptimizationParams(OptimizationParams optimizationParams) { this->optimizationParams = optimizationParams; }
  void setRelinearizeThreshold(RelinearizationThreshold relinearizeThreshold) { this->relinearizeThreshold = relinearizeThreshold; }
  void setRelinearizeSkip(int relinearizeSkip) { this->relinearizeSkip = relinearizeSkip; }
  void setEnableRelinearization(bool enableRelinearization) { this->enableRelinearization = enableRelinearization; }
  void setEvaluateNonlinearError(bool evaluateNonlinearError) { this->evaluateNonlinearError = evaluateNonlinearError; }
  void setFactorization(const std::string& factorization) { this->factorization = factorizationTranslator(factorization); }
  void setCacheLinearizedFactors(bool cacheLinearizedFactors) { this->cacheLinearizedFactors = cacheLinearizedFactors; }
  void setKeyFormatter(KeyFormatter keyFormatter) { this->keyFormatter = keyFormatter; }
  void setEnableDetailedResults(bool enableDetailedResults) { this->enableDetailedResults = enableDetailedResults; }
  void setEnablePartialRelinearizationCheck(bool enablePartialRelinearizationCheck) { this->enablePartialRelinearizationCheck = enablePartialRelinearizationCheck; }

  GaussianFactorGraph::Eliminate getEliminationFunction() const {
    return factorization == CHOLESKY
      ? (GaussianFactorGraph::Eliminate)EliminatePreferCholesky
      : (GaussianFactorGraph::Eliminate)EliminateQR;
  }

  /// @}

  /// @name Some utilities
  /// @{

  static Factorization factorizationTranslator(const std::string& str);
  static std::string factorizationTranslator(const Factorization& value);

  /// @}
};

typedef FastVector<size_t> FactorIndices;

/**
 * @addtogroup ISAM2
 * This struct is returned from ISAM2::update() and contains information about
 * the update that is useful for determining whether the solution is
 * converging, and about how much work was required for the update.  See member
 * variables for details and information about each entry.
 */
struct GTSAM_EXPORT ISAM2Result {
  /** The nonlinear error of all of the factors, \a including new factors and
   * variables added during the current call to ISAM2::update().  This error is
   * calculated using the following variable values:
   * \li Pre-existing variables will be evaluated by combining their
   * linearization point before this call to update, with their partial linear
   * delta, as computed by ISAM2::calculateEstimate().
   * \li New variables will be evaluated at their initialization points passed
   * into the current call to update.
   * \par Note: This will only be computed if ISAM2Params::evaluateNonlinearError
   * is set to \c true, because there is some cost to this computation.
   */
  boost::optional<double> errorBefore;

  /** The nonlinear error of all of the factors computed after the current
   * update, meaning that variables above the relinearization threshold
   * (ISAM2Params::relinearizeThreshold) have been relinearized and new
   * variables have undergone one linear update.  Variable values are
   * again computed by combining their linearization points with their
   * partial linear deltas, by ISAM2::calculateEstimate().
   * \par Note: This will only be computed if ISAM2Params::evaluateNonlinearError
   * is set to \c true, because there is some cost to this computation.
   */
  boost::optional<double> errorAfter;

  /** The number of variables that were relinearized because their linear
   * deltas exceeded the reslinearization threshold
   * (ISAM2Params::relinearizeThreshold), combined with any additional
   * variables that had to be relinearized because they were involved in
   * the same factor as a variable above the relinearization threshold.
   * On steps where no relinearization is considered
   * (see ISAM2Params::relinearizeSkip), this count will be zero.
   */
  size_t variablesRelinearized;

  /** The number of variables that were reeliminated as parts of the Bayes'
   * Tree were recalculated, due to new factors.  When loop closures occur,
   * this count will be large as the new loop-closing factors will tend to
   * involve variables far away from the root, and everything up to the root
   * will be reeliminated.
   */
  size_t variablesReeliminated;

  /** The number of factors that were included in reelimination of the Bayes' tree. */
  size_t factorsRecalculated;

  /** The number of cliques in the Bayes' Tree */
  size_t cliques;

  /** The indices of the newly-added factors, in 1-to-1 correspondence with the
   * factors passed as \c newFactors to ISAM2::update().  These indices may be
   * used later to refer to the factors in order to remove them.
   */
  FactorIndices newFactorsIndices;

  /** A struct holding detailed results, which must be enabled with
   * ISAM2Params::enableDetailedResults.
   */
  struct DetailedResults {
    /** The status of a single variable, this struct is stored in
     * DetailedResults::variableStatus */
    struct VariableStatus {
      /** Whether the variable was just reeliminated, due to being relinearized,
       * observed, new, or on the path up to the root clique from another
       * reeliminated variable. */
      bool isReeliminated;
      bool isAboveRelinThreshold; ///< Whether the variable was just relinearized due to being above the relinearization threshold
      bool isRelinearizeInvolved; ///< Whether the variable was below the relinearization threshold but was relinearized by being involved in a factor with a variable above the relinearization threshold
      bool isRelinearized; /// Whether the variable was relinearized, either by being above the relinearization threshold or by involvement.
      bool isObserved; ///< Whether the variable was just involved in new factors
      bool isNew; ///< Whether the variable itself was just added
      bool inRootClique; ///< Whether the variable is in the root clique
      VariableStatus(): isReeliminated(false), isAboveRelinThreshold(false), isRelinearizeInvolved(false),
          isRelinearized(false), isObserved(false), isNew(false), inRootClique(false) {}
    };

    /** The status of each variable during this update, see VariableStatus.
     */
    FastMap<Key, VariableStatus> variableStatus;
  };

  /** Detailed results, if enabled by ISAM2Params::enableDetailedResults.  See
   * Detail for information about the results data stored here. */
  boost::optional<DetailedResults> detail;


  void print(const std::string str = "") const {
    std::cout << str << "  Reelimintated: " << variablesReeliminated << "  Relinearized: " << variablesRelinearized << "  Cliques: " << cliques << std::endl;
  }

  /** Getters and Setters */
  size_t getVariablesRelinearized() const { return variablesRelinearized; };
  size_t getVariablesReeliminated() const { return variablesReeliminated; };
  size_t getCliques() const { return cliques; };
};

/**
 * Specialized Clique structure for ISAM2, incorporating caching and gradient contribution
 * TODO: more documentation
 */
class GTSAM_EXPORT ISAM2Clique : public BayesTreeCliqueBase<ISAM2Clique, GaussianFactorGraph>
{
public:
  typedef ISAM2Clique This;
  typedef BayesTreeCliqueBase<This, GaussianFactorGraph> Base;
  typedef boost::shared_ptr<This> shared_ptr;
  typedef boost::weak_ptr<This> weak_ptr;
  typedef GaussianConditional ConditionalType;
  typedef ConditionalType::shared_ptr sharedConditional;

  Base::FactorType::shared_ptr cachedFactor_;
  Vector gradientContribution_;
  FastMap<Key, VectorValues::iterator> solnPointers_;

  /// Default constructor
  ISAM2Clique() : Base() {}

  /// Copy constructor, does *not* copy solution pointers as these are invalid in different trees.
  ISAM2Clique(const ISAM2Clique& other) :
    Base(other), cachedFactor_(other.cachedFactor_), gradientContribution_(other.gradientContribution_) {}

  /// Assignment operator, does *not* copy solution pointers as these are invalid in different trees.
  ISAM2Clique& operator=(const ISAM2Clique& other)
  {
    Base::operator=(other);
    cachedFactor_ = other.cachedFactor_;
    gradientContribution_ = other.gradientContribution_;
    return *this;
  }

  /// Overridden to also store the remaining factor and gradient contribution
  void setEliminationResult(const FactorGraphType::EliminationResult& eliminationResult);

  /** Access the cached factor */
  Base::FactorType::shared_ptr& cachedFactor() { return cachedFactor_; }

  /** Access the gradient contribution */
  const Vector& gradientContribution() const { return gradientContribution_; }

  bool equals(const This& other, double tol=1e-9) const;

  /** print this node */
  void print(const std::string& s = "", const KeyFormatter& formatter = DefaultKeyFormatter) const;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(cachedFactor_);
    ar & BOOST_SERIALIZATION_NVP(gradientContribution_);
  }
}; // \struct ISAM2Clique

/**
 * @addtogroup ISAM2
 * Implementation of the full ISAM2 algorithm for incremental nonlinear optimization.
 *
 * The typical cycle of using this class to create an instance by providing ISAM2Params
 * to the constructor, then add measurements and variables as they arrive using the update()
 * method.  At any time, calculateEstimate() may be called to obtain the current
 * estimate of all variables.
 *
 */
class GTSAM_EXPORT ISAM2: public BayesTree<ISAM2Clique> {

protected:

  /** The current linearization point */
  Values theta_;

  /** VariableIndex lets us look up factors by involved variable and keeps track of dimensions */
  VariableIndex variableIndex_;

  /** The linear delta from the last linear solution, an update to the estimate in theta
   *
   * This is \c mutable because it is a "cached" variable - it is not updated
   * until either requested with getDelta() or calculateEstimate(), or needed
   * during update() to evaluate whether to relinearize variables.
   */
  mutable VectorValues delta_;

  mutable VectorValues deltaNewton_; // Only used when using Dogleg - stores the Gauss-Newton update
  mutable VectorValues RgProd_; // Only used when using Dogleg - stores R*g and is updated incrementally

  /** A cumulative mask for the variables that were replaced and have not yet
   * been updated in the linear solution delta_, this is only used internally,
   * delta will always be updated if necessary when requested with getDelta()
   * or calculateEstimate().
   *
   * This is \c mutable because it is used internally to not update delta_
   * until it is needed.
   */
  mutable KeySet deltaReplacedMask_; // TODO: Make sure accessed in the right way

  /** All original nonlinear factors are stored here to use during relinearization */
  NonlinearFactorGraph nonlinearFactors_;

  /** The current linear factors, which are only updated as needed */
  mutable GaussianFactorGraph linearFactors_;

  /** The current parameters */
  ISAM2Params params_;

  /** The current Dogleg Delta (trust region radius) */
  mutable boost::optional<double> doglegDelta_;

  /** Set of variables that are involved with linear factors from marginalized
   * variables and thus cannot have their linearization points changed. */
  KeySet fixedVariables_;

  int update_count_; ///< Counter incremented every update(), used to determine periodic relinearization

public:

  typedef ISAM2 This; ///< This class
  typedef BayesTree<ISAM2Clique> Base; ///< The BayesTree base class
  typedef Base::Clique Clique; ///< A clique
  typedef Base::sharedClique sharedClique; ///< Shared pointer to a clique
  typedef Base::Cliques Cliques; ///< List of Clique typedef from base class

  /** Create an empty ISAM2 instance */
  ISAM2(const ISAM2Params& params);

  /** Create an empty ISAM2 instance using the default set of parameters (see ISAM2Params) */
  ISAM2();

  /** default virtual destructor */
  virtual ~ISAM2() {}

  /** Compare equality */
  virtual bool equals(const ISAM2& other, double tol = 1e-9) const;

  /**
   * Add new factors, updating the solution and relinearizing as needed.
   *
   * Optionally, this function remove existing factors from the system to enable
   * behaviors such as swapping existing factors with new ones.
   *
   * Add new measurements, and optionally new variables, to the current system.
   * This runs a full step of the ISAM2 algorithm, relinearizing and updating
   * the solution as needed, according to the wildfire and relinearize
   * thresholds.
   *
   * @param newFactors The new factors to be added to the system
   * @param newTheta Initialization points for new variables to be added to the system.
   * You must include here all new variables occuring in newFactors (which were not already
   * in the system).  There must not be any variables here that do not occur in newFactors,
   * and additionally, variables that were already in the system must not be included here.
   * @param removeFactorIndices Indices of factors to remove from system
   * @param force_relinearize Relinearize any variables whose delta magnitude is sufficiently
   * large (Params::relinearizeThreshold), regardless of the relinearization interval
   * (Params::relinearizeSkip).
   * @param constrainedKeys is an optional map of keys to group labels, such that a variable can
   * be constrained to a particular grouping in the BayesTree
   * @param noRelinKeys is an optional set of nonlinear keys that iSAM2 will hold at a constant linearization
   * point, regardless of the size of the linear delta
   * @param extraReelimKeys is an optional set of nonlinear keys that iSAM2 will re-eliminate, regardless
   * of the size of the linear delta. This allows the provided keys to be reordered.
   * @return An ISAM2Result struct containing information about the update
   */
  virtual ISAM2Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
      const Values& newTheta = Values(),
      const FactorIndices& removeFactorIndices = FactorIndices(),
      const boost::optional<FastMap<Key,int> >& constrainedKeys = boost::none,
      const boost::optional<FastList<Key> >& noRelinKeys = boost::none,
      const boost::optional<FastList<Key> >& extraReelimKeys = boost::none,
      bool force_relinearize = false);

  /** Marginalize out variables listed in leafKeys.  These keys must be leaves
   * in the BayesTree.  Throws MarginalizeNonleafException if non-leaves are
   * requested to be marginalized.  Marginalization leaves a linear
   * approximation of the marginal in the system, and the linearization points
   * of any variables involved in this linear marginal become fixed.  The set
   * fixed variables will include any key involved with the marginalized variables
   * in the original factors, and possibly additional ones due to fill-in.
   *
   * If provided, 'marginalFactorsIndices' will be augmented with the factor graph
   * indices of the marginal factors added during the 'marginalizeLeaves' call
   *
   * If provided, 'deletedFactorsIndices' will be augmented with the factor graph
   * indices of any factor that was removed during the 'marginalizeLeaves' call
   */
  void marginalizeLeaves(const FastList<Key>& leafKeys,
    boost::optional<FactorIndices&> marginalFactorsIndices = boost::none,
    boost::optional<FactorIndices&> deletedFactorsIndices = boost::none);

  /// Access the current linearization point
  const Values& getLinearizationPoint() const {
    return theta_;
  }

  /// Check whether variable with given key exists in linearization point
  bool valueExists(Key key) const {
    return theta_.exists(key);
  }

  /** Compute an estimate from the incomplete linear delta computed during the last update.
   * This delta is incomplete because it was not updated below wildfire_threshold.  If only
   * a single variable is needed, it is faster to call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const;

  /** Compute an estimate for a single variable using its incomplete linear delta computed
   * during the last update.  This is faster than calling the no-argument version of
   * calculateEstimate, which operates on all variables.
   * @param key
   * @return
   */
  template<class VALUE>
  VALUE calculateEstimate(Key key) const;

  /** Compute an estimate for a single variable using its incomplete linear delta computed
   * during the last update.  This is faster than calling the no-argument version of
   * calculateEstimate, which operates on all variables.  This is a non-templated version
   * that returns a Value base class for use with the MATLAB wrapper.
   * @param key
   * @return
   */
  const Value& calculateEstimate(Key key) const;

  /** Return marginal on any variable as a covariance matrix */
  Matrix marginalCovariance(Key key) const;

  /// @name Public members for non-typical usage
  /// @{

  /** Internal implementation functions */
  struct Impl;

  /** Compute an estimate using a complete delta computed by a full back-substitution.
   */
  Values calculateBestEstimate() const;

  /** Access the current delta, computed during the last call to update */
  const VectorValues& getDelta() const;

  /** Compute the linear error */
  double error(const VectorValues& x) const;

  /** Access the set of nonlinear factors */
  const NonlinearFactorGraph& getFactorsUnsafe() const { return nonlinearFactors_; }

  /** Access the nonlinear variable index */
  const VariableIndex& getVariableIndex() const { return variableIndex_; }

  /** Access the nonlinear variable index */
  const KeySet& getFixedVariables() const { return fixedVariables_; }

  size_t lastAffectedVariableCount;
  size_t lastAffectedFactorCount;
  size_t lastAffectedCliqueCount;
  size_t lastAffectedMarkedCount;
  mutable size_t lastBacksubVariableCount;
  size_t lastNnzTop;

  const ISAM2Params& params() const { return params_; }

  /** prints out clique statistics */
  void printStats() const { getCliqueData().getStats().print(); }
  
  /** Compute the gradient of the energy function, \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} R x - d
   * \right\Vert^2 \f$, centered around zero. The gradient about zero is \f$ -R^T d \f$.  See also
   * gradient(const GaussianBayesNet&, const VectorValues&).
   *
   * @return A VectorValues storing the gradient.
   */
  VectorValues gradientAtZero() const;
  
  /// @}

protected:

  FastSet<Key> getAffectedFactors(const FastList<Key>& keys) const;
  GaussianFactorGraph::shared_ptr relinearizeAffectedFactors(const FastList<Key>& affectedKeys, const KeySet& relinKeys) const;
  GaussianFactorGraph getCachedBoundaryFactors(Cliques& orphans);

  virtual boost::shared_ptr<KeySet > recalculate(const KeySet& markedKeys, const KeySet& relinKeys,
      const std::vector<Key>& observedKeys, const KeySet& unusedIndices, const boost::optional<FastMap<Key,int> >& constrainKeys, ISAM2Result& result);
  void updateDelta(bool forceFullSolve = false) const;

}; // ISAM2

/// traits
template<> struct traits<ISAM2> : public Testable<ISAM2> {};

/// Optimize the BayesTree, starting from the root.
/// @param replaced Needs to contain
/// all variables that are contained in the top of the Bayes tree that has been
/// redone.
/// @param delta The current solution, an offset from the linearization
/// point.
/// @param threshold The maximum change against the PREVIOUS delta for
/// non-replaced variables that can be ignored, ie. the old delta entry is kept
/// and recursive backsubstitution might eventually stop if none of the changed
/// variables are contained in the subtree.
/// @return The number of variables that were solved for
template<class CLIQUE>
size_t optimizeWildfire(const boost::shared_ptr<CLIQUE>& root,
    double threshold, const KeySet& replaced, VectorValues& delta);

template<class CLIQUE>
size_t optimizeWildfireNonRecursive(const boost::shared_ptr<CLIQUE>& root,
    double threshold, const KeySet& replaced, VectorValues& delta);

/// calculate the number of non-zero entries for the tree starting at clique (use root for complete matrix)
template<class CLIQUE>
int calculate_nnz(const boost::shared_ptr<CLIQUE>& clique);

} /// namespace gtsam

#include <gtsam/nonlinear/ISAM2-inl.h>
#include <gtsam/nonlinear/ISAM2-impl.h>
