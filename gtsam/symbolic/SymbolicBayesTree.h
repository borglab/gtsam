/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicBayesTree.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>

namespace gtsam {

  // Forward declarations
  class SymbolicConditional;

  /* ************************************************************************* */
  /// A clique in a SymbolicBayesTree
  class GTSAM_EXPORT SymbolicBayesTreeClique :
    public BayesTreeCliqueBase<SymbolicBayesTreeClique, SymbolicFactorGraph>
  {
  public:
    typedef SymbolicBayesTreeClique This;
    typedef BayesTreeCliqueBase<SymbolicBayesTreeClique, SymbolicFactorGraph> Base;
    typedef std::shared_ptr<This> shared_ptr;
    typedef std::weak_ptr<This> weak_ptr;
    SymbolicBayesTreeClique() {}
    SymbolicBayesTreeClique(const std::shared_ptr<SymbolicConditional>& conditional) : Base(conditional) {}
  };

  /* ************************************************************************* */
  /// A Bayes tree that represents the connectivity between variables but is not associated with any
  /// probability functions.
  class GTSAM_EXPORT SymbolicBayesTree :
    public BayesTree<SymbolicBayesTreeClique>
  {
  private:
    typedef BayesTree<SymbolicBayesTreeClique> Base;

  public:
    typedef SymbolicBayesTree This;
    typedef std::shared_ptr<This> shared_ptr;

    /** Default constructor, creates an empty Bayes tree */
    SymbolicBayesTree() {}

    /** check equality */
    bool equals(const This& other, double tol = 1e-9) const;

  private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
#endif
  };

/// traits
template<> struct traits<SymbolicBayesTreeClique> : public Testable<SymbolicBayesTreeClique> {};
template<> struct traits<SymbolicBayesTree> : public Testable<SymbolicBayesTree> {};

} //\ namespace gtsam

