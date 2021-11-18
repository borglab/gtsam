/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file Constraint.h
 *  @date May 15, 2012
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <boost/assign.hpp>

namespace gtsam {

  class Domain;

  /**
   * Base class for constraint factors
   * Derived classes include SingleValue, BinaryAllDiff, and AllDiff.
   */
  class GTSAM_EXPORT Constraint : public DiscreteFactor {

  public:

    typedef boost::shared_ptr<Constraint> shared_ptr;

  protected:

    /// Construct unary constraint factor.
    Constraint(Key j) :
      DiscreteFactor(boost::assign::cref_list_of<1>(j)) {
    }

    /// Construct binary constraint factor.
    Constraint(Key j1, Key j2) :
      DiscreteFactor(boost::assign::cref_list_of<2>(j1)(j2)) {
    }

    /// Construct n-way constraint factor.
    Constraint(const KeyVector& js) :
      DiscreteFactor(js) {
    }

    /// construct from container
    template<class KeyIterator>
    Constraint(KeyIterator beginKey, KeyIterator endKey) :
      DiscreteFactor(beginKey, endKey) {
    }

  public:

    /// @name Standard Constructors
    /// @{

    /// Default constructor for I/O
    Constraint();

    /// Virtual destructor
    ~Constraint() override {}

    /// @}
    /// @name Standard Interface
    /// @{

    /*
     * Ensure Arc-consistency, possibly changing domains of connected variables.
     * @param j domain to be checked
     * @param (in/out) domains all other domains
     * @return true if domains were changed, false otherwise.
     */
    virtual bool ensureArcConsistency(size_t j,
                                      std::vector<Domain>* domains) const = 0;

    /// Partially apply known values
    virtual shared_ptr partiallyApply(const Values&) const = 0;


    /// Partially apply known values, domain version
    virtual shared_ptr partiallyApply(const std::vector<Domain>&) const = 0;
    /// @}
  };
// DiscreteFactor

}// namespace gtsam
