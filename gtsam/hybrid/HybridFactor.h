/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridFactor.h
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/base/Testable.h>

#include <string>
namespace gtsam {

KeyVector CollectKeys(const KeyVector &continuousKeys, const DiscreteKeys &discreteKeys);
KeyVector CollectKeys(const KeyVector &keys1, const KeyVector &keys2);

/**
 * Base class for hybrid probabilistic factors
 */
class GTSAM_EXPORT HybridFactor : public Factor {

public:

  // typedefs needed to play nice with gtsam
  typedef HybridFactor This; ///< This class
  typedef boost::shared_ptr<HybridFactor> shared_ptr; ///< shared_ptr to this class
  typedef Factor Base; ///< Our base class

  bool isDiscrete_ = false;
  bool isContinuous_ = false;
  bool isHybrid_ = false;

  DiscreteKeys discreteKeys_;

public:

/// @name Standard Constructors
/// @{

  /** Default constructor creates empty factor */
  HybridFactor();

  /** Construct from container of keys.  This constructor is used internally from derived factor
   *  constructors, either from a container of keys or from a boost::assign::list_of. */
//  template<typename CONTAINER>
//  HybridFactor(const CONTAINER &keys) : Base(keys) {}

  explicit HybridFactor(const KeyVector &keys);

  HybridFactor(const KeyVector &continuousKeys, const DiscreteKeys &discreteKeys);

  explicit HybridFactor(const DiscreteKeys &discreteKeys);

  /// Virtual destructor
  virtual ~HybridFactor();

/// @}
/// @name Testable
/// @{

  /// equals
  virtual bool equals(const HybridFactor &lf, double tol = 1e-9) const = 0;

  /// print
  void print(
      const std::string &s = "HybridFactor\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override {
    std::cout << s;
    if (isContinuous_) std::cout << "Cont. ";
    if (isDiscrete_) std::cout << "Disc. ";
    if (isHybrid_) std::cout << "Hybr. ";
    this->printKeys("", formatter);
  }

/// @}
/// @name Standard Interface
/// @{

/// @}
};
// HybridFactor

// traits
template<>
struct traits<HybridFactor> : public Testable<HybridFactor> {};

}// namespace gtsam
