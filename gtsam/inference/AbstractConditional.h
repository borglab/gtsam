/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AbstractConditional.h
 * @brief   Abstract base class for conditional densities
 * @author  Fan Jiang
 */

// \callgraph
#pragma once

#include <gtsam/inference/Key.h>

#include <boost/range.hpp>

namespace gtsam {

class GTSAM_EXPORT AbstractConditional {
 public:
  using shared_ptr = boost::shared_ptr<AbstractConditional>;

 protected:
  /** The first nrFrontal variables are frontal and the rest are parents. */
  size_t nrFrontals_;

 public:
  /** View of the frontal keys (call frontals()) */
  typedef boost::iterator_range<typename KeyVector::const_iterator> Frontals;

  /** View of the separator keys (call parents()) */
  typedef boost::iterator_range<typename KeyVector::const_iterator> Parents;

 protected:
  /// @name Standard Constructors
  /// @{

  /** Empty Constructor to make serialization possible */
  AbstractConditional() : nrFrontals_(0) {}

  /** Constructor */
  AbstractConditional(size_t nrFrontals) : nrFrontals_(nrFrontals) {}

  /// @}

 public:
  virtual ~AbstractConditional() = default;

  /// @name Testable
  /// @{

  /** print with optional formatter */
  virtual void print(const std::string &s = "AbstractConditional",
                     const KeyFormatter &formatter = DefaultKeyFormatter) const;

  /** check equality */
  bool equals(const AbstractConditional &c, double tol = 1e-9) const;

  /// @}

  /// @name Standard Interface
  /// @{

  /** return the number of frontals */
  size_t nrFrontals() const { return nrFrontals_; }

  /** Mutable version of nrFrontals */
  size_t &nrFrontals() { return nrFrontals_; }

  /** return the number of parents */
  virtual size_t nrParents() const = 0;

  /** return a view of the frontal keys */
  virtual Frontals frontals() const = 0;

  virtual Parents parents() const = 0;
  /// @}

  /** Iterator at beginning of involved variable keys */
  KeyVector::const_iterator begin() const {
    throw std::runtime_error("AbstractConditional::begin not implemented!");
  }

  /** Iterator at end of involved variable keys */
  KeyVector::const_iterator end() const {
    throw std::runtime_error("AbstractConditional::end not implemented!");
  }
};

/// traits
template <>
struct traits<AbstractConditional> : public Testable<AbstractConditional> {};

}  // namespace gtsam
