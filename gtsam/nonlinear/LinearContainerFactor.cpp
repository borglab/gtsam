/**
 * @file LinearContainerFactor.cpp
 *
 * @date Jul 6, 2012
 * @author Alex Cunningham
 */

#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

/* ************************************************************************* */
void LinearContainerFactor::initializeLinearizationPoint(const Values& linearizationPoint) {
  if (!linearizationPoint.empty()) {
    linearizationPoint_ = Values();
    for (Key key : keys()) {
      linearizationPoint_->insert(key, linearizationPoint.at(key));
    }
  } else {
    linearizationPoint_ = boost::none;
  }
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(const GaussianFactor::shared_ptr& factor,
    const boost::optional<Values>& linearizationPoint)
: NonlinearFactor(factor->keys()), factor_(factor), linearizationPoint_(linearizationPoint) {
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
    const JacobianFactor& factor, const Values& linearizationPoint)
: NonlinearFactor(factor.keys()), factor_(factor.clone()) {
  initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
    const HessianFactor& factor, const Values& linearizationPoint)
: NonlinearFactor(factor.keys()), factor_(factor.clone()) {
  initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
    const GaussianFactor::shared_ptr& factor, const Values& linearizationPoint)
: NonlinearFactor(factor->keys()), factor_(factor->clone()) {
  initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
void LinearContainerFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s+"LinearContainerFactor", keyFormatter);
  if (factor_)
    factor_->print("   Stored Factor", keyFormatter);
  if (linearizationPoint_)
    linearizationPoint_->print("   LinearizationPoint", keyFormatter);
}

/* ************************************************************************* */
bool LinearContainerFactor::equals(const NonlinearFactor& f, double tol) const {
  const LinearContainerFactor* jcf = dynamic_cast<const LinearContainerFactor*>(&f);
  if (!jcf || !factor_->equals(*jcf->factor_, tol) || !NonlinearFactor::equals(f))
    return false;
  if (!linearizationPoint_ && !jcf->linearizationPoint_)
    return true;
  if (linearizationPoint_ && jcf->linearizationPoint_)
    return linearizationPoint_->equals(*jcf->linearizationPoint_, tol);
  return false;
}

/* ************************************************************************* */
double LinearContainerFactor::error(const Values& c) const {
  if (!linearizationPoint_)
    return 0;

  // Extract subset of values for comparison
  Values csub;
  for (Key key : keys())
    csub.insert(key, c.at(key));

  // create dummy ordering for evaluation
  VectorValues delta = linearizationPoint_->localCoordinates(csub);

  // compute error
  double error = factor_->error(delta);

  return error;
}

/* ************************************************************************* */
size_t LinearContainerFactor::dim() const {
  if (isJacobian())
    return toJacobian()->get_model()->dim();
  else
    return 1; // Hessians don't have true dimension
}

/* ************************************************************************* */
GaussianFactor::shared_ptr LinearContainerFactor::linearize(const Values& c) const {
  // Clone factor and update as necessary
  GaussianFactor::shared_ptr linFactor = factor_->clone();
  if (!hasLinearizationPoint())
    return linFactor;

  // Extract subset of values
  Values subsetC;
  for (Key key : keys())
    subsetC.insert(key, c.at(key));

  // Determine delta between linearization points using new ordering
  VectorValues delta = linearizationPoint_->localCoordinates(subsetC);

  // Apply changes due to relinearization
  if (isJacobian()) {
    JacobianFactor::shared_ptr jacFactor = boost::dynamic_pointer_cast<JacobianFactor>(linFactor);
    jacFactor->getb() = -jacFactor->unweighted_error(delta);
  } else {
    HessianFactor::shared_ptr hesFactor = boost::dynamic_pointer_cast<HessianFactor>(linFactor);

    const auto view = hesFactor->informationView();
    Vector deltaVector = delta.vector(keys());
    Vector G_delta = view * deltaVector;
    hesFactor->constantTerm() += deltaVector.dot(G_delta) - 2.0 * deltaVector.dot(hesFactor->linearTerm().col(0));
    hesFactor->linearTerm() -= G_delta;
  }

  return linFactor;
}

/* ************************************************************************* */
bool LinearContainerFactor::isJacobian() const {
  return dynamic_cast<const JacobianFactor*>(factor_.get()) != 0;
}

/* ************************************************************************* */
bool LinearContainerFactor::isHessian() const {
  return dynamic_cast<const HessianFactor*>(factor_.get()) != 0;
}

/* ************************************************************************* */
JacobianFactor::shared_ptr LinearContainerFactor::toJacobian() const {
  return boost::dynamic_pointer_cast<JacobianFactor>(factor_);
}

/* ************************************************************************* */
HessianFactor::shared_ptr LinearContainerFactor::toHessian() const {
  return boost::dynamic_pointer_cast<HessianFactor>(factor_);
}

/* ************************************************************************* */
GaussianFactor::shared_ptr LinearContainerFactor::negateToGaussian() const {
  GaussianFactor::shared_ptr result = factor_->negate();
  return result;
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr LinearContainerFactor::negateToNonlinear() const {
  GaussianFactor::shared_ptr antifactor = factor_->negate(); // already has keys in place
  return NonlinearFactor::shared_ptr(new LinearContainerFactor(antifactor, linearizationPoint_));
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr LinearContainerFactor::rekey(
    const std::map<Key, Key>& rekey_mapping) const {
  auto rekeyed_base_factor = Base::rekey(rekey_mapping);
  // Update the keys to the properties as well
  // Downncast so we have access to members
  auto new_factor = boost::static_pointer_cast<LinearContainerFactor>(rekeyed_base_factor);
  // Create a new Values to assign later
  Values newLinearizationPoint;
  for (size_t i = 0; i < factor_->size(); ++i) {
    auto mapping = rekey_mapping.find(factor_->keys()[i]);
    if (mapping != rekey_mapping.end()) {
      new_factor->factor_->keys()[i] = mapping->second;
      newLinearizationPoint.insert(mapping->second, linearizationPoint_->at(mapping->first));
    }
  }
  new_factor->linearizationPoint_ = newLinearizationPoint;

  // upcast back and return
  return boost::static_pointer_cast<NonlinearFactor>(new_factor);
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr LinearContainerFactor::rekey(
    const KeyVector& new_keys) const {
  auto rekeyed_base_factor = Base::rekey(new_keys);
  // Update the keys to the properties as well
  // Downncast so we have access to members
  auto new_factor = boost::static_pointer_cast<LinearContainerFactor>(rekeyed_base_factor);
  new_factor->factor_->keys() = new_factor->keys();
  // Create a new Values to assign later
  Values newLinearizationPoint;
  for(size_t i=0; i<new_keys.size(); ++i) {
    Key cur_key = linearizationPoint_->keys()[i];
    newLinearizationPoint.insert(new_keys[i], linearizationPoint_->at(cur_key));
  }
  new_factor->linearizationPoint_ = newLinearizationPoint;

  // upcast back and return
  return boost::static_pointer_cast<NonlinearFactor>(new_factor);
}

/* ************************************************************************* */
NonlinearFactorGraph LinearContainerFactor::ConvertLinearGraph(
    const GaussianFactorGraph& linear_graph, const Values& linearizationPoint) {
  NonlinearFactorGraph result;
  result.reserve(linear_graph.size());
  for (const auto& f : linear_graph)
    if (f)
      result += boost::make_shared<LinearContainerFactor>(f, linearizationPoint);
  return result;
}

/* ************************************************************************* */
} // \namespace gtsam

