/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridValues.cpp
 * @author  Varun Agrawal
 * @date    August 2024
 */

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

/* ************************************************************************* */
HybridValues::HybridValues(const VectorValues& cv, const DiscreteValues& dv)
    : continuous_(cv), discrete_(dv) {}

/* ************************************************************************* */
HybridValues::HybridValues(const VectorValues& cv, const DiscreteValues& dv,
                           const Values& v)
    : continuous_(cv), discrete_(dv), nonlinear_(v) {}

/* ************************************************************************* */
void HybridValues::print(const std::string& s,
                         const KeyFormatter& keyFormatter) const {
  std::cout << s << ": \n";
  // print continuous components
  continuous_.print("  Continuous", keyFormatter);
  // print discrete components
  discrete_.print("  Discrete", keyFormatter);
  // print nonlinear components
  nonlinear_.print("  Nonlinear", keyFormatter);
}

/* ************************************************************************* */
bool HybridValues::equals(const HybridValues& other, double tol) const {
  return continuous_.equals(other.continuous_, tol) &&
         discrete_.equals(other.discrete_, tol);
}

/* ************************************************************************* */
const VectorValues& HybridValues::continuous() const { return continuous_; }

/* ************************************************************************* */
const DiscreteValues& HybridValues::discrete() const { return discrete_; }

/* ************************************************************************* */
const Values& HybridValues::nonlinear() const { return nonlinear_; }

/* ************************************************************************* */
bool HybridValues::existsVector(Key j) { return continuous_.exists(j); }

/* ************************************************************************* */
bool HybridValues::existsDiscrete(Key j) {
  return (discrete_.find(j) != discrete_.end());
}

/* ************************************************************************* */
bool HybridValues::existsNonlinear(Key j) { return nonlinear_.exists(j); }

/* ************************************************************************* */
bool HybridValues::exists(Key j) {
  return existsVector(j) || existsDiscrete(j) || existsNonlinear(j);
}

/* ************************************************************************* */
HybridValues HybridValues::retract(const VectorValues& delta) const {
  HybridValues updated(continuous_, discrete_, nonlinear_.retract(delta));
  return updated;
}

/* ************************************************************************* */
void HybridValues::insert(Key j, const Vector& value) {
  continuous_.insert(j, value);
}

/* ************************************************************************* */
void HybridValues::insert(Key j, size_t value) { discrete_[j] = value; }

/* ************************************************************************* */
void HybridValues::insert_or_assign(Key j, const Vector& value) {
  continuous_.insert_or_assign(j, value);
}

/* ************************************************************************* */
void HybridValues::insert_or_assign(Key j, size_t value) {
  discrete_[j] = value;
}

/* ************************************************************************* */
HybridValues& HybridValues::insert(const VectorValues& values) {
  continuous_.insert(values);
  return *this;
}

/* ************************************************************************* */
HybridValues& HybridValues::insert(const DiscreteValues& values) {
  discrete_.insert(values);
  return *this;
}

/* ************************************************************************* */
HybridValues& HybridValues::insert(const Values& values) {
  nonlinear_.insert(values);
  return *this;
}

/* ************************************************************************* */
HybridValues& HybridValues::insert(const HybridValues& values) {
  continuous_.insert(values.continuous());
  discrete_.insert(values.discrete());
  nonlinear_.insert(values.nonlinear());
  return *this;
}

/* ************************************************************************* */
Vector& HybridValues::at(Key j) { return continuous_.at(j); }

/* ************************************************************************* */
size_t& HybridValues::atDiscrete(Key j) { return discrete_.at(j); }

/* ************************************************************************* */
HybridValues& HybridValues::update(const VectorValues& values) {
  continuous_.update(values);
  return *this;
}

/* ************************************************************************* */
HybridValues& HybridValues::update(const DiscreteValues& values) {
  discrete_.update(values);
  return *this;
}

/* ************************************************************************* */
HybridValues& HybridValues::update(const HybridValues& values) {
  continuous_.update(values.continuous());
  discrete_.update(values.discrete());
  return *this;
}

/* ************************************************************************* */
VectorValues HybridValues::continuousSubset(const KeyVector& keys) const {
  VectorValues measurements;
  for (const auto& key : keys) {
    measurements.insert(key, continuous_.at(key));
  }
  return measurements;
}

/* ************************************************************************* */
std::string HybridValues::html(const KeyFormatter& keyFormatter) const {
  std::stringstream ss;
  ss << this->continuous_.html(keyFormatter);
  ss << this->discrete_.html(keyFormatter);
  return ss.str();
}

}  // namespace gtsam
