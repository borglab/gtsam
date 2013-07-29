/**
 * @file LinearContainerFactor.cpp
 *
 * @date Jul 6, 2012
 * @author Alex Cunningham
 */

#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <boost/foreach.hpp>

namespace gtsam {

/* ************************************************************************* */
void LinearContainerFactor::rekeyFactor(const OrderingOrdered& ordering) {
  BOOST_FOREACH(Index& idx, factor_->keys()) {
    Key fullKey = ordering.key(idx);
    idx = fullKey;
    keys_.push_back(fullKey);
  }
}

/* ************************************************************************* */
void LinearContainerFactor::initializeLinearizationPoint(const Values& linearizationPoint) {
  if (!linearizationPoint.empty()) {
    linearizationPoint_ = Values();
    BOOST_FOREACH(const gtsam::Key& key, this->keys()) {
      linearizationPoint_->insert(key, linearizationPoint.at(key));
    }
  } else {
    linearizationPoint_ = boost::none;
  }
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(const GaussianFactorOrdered::shared_ptr& factor,
    const boost::optional<Values>& linearizationPoint)
: factor_(factor), linearizationPoint_(linearizationPoint) {
  // Extract keys stashed in linear factor
  BOOST_FOREACH(const Index& idx, factor_->keys())
    keys_.push_back(idx);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
    const JacobianFactorOrdered& factor, const OrderingOrdered& ordering,
    const Values& linearizationPoint)
: factor_(factor.clone()) {
  rekeyFactor(ordering);
  initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
    const HessianFactorOrdered& factor, const OrderingOrdered& ordering,
    const Values& linearizationPoint)
: factor_(factor.clone()) {
  rekeyFactor(ordering);
  initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
    const GaussianFactorOrdered::shared_ptr& factor, const OrderingOrdered& ordering,
    const Values& linearizationPoint)
: factor_(factor->clone()) {
  rekeyFactor(ordering);
  initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
    const GaussianFactorOrdered::shared_ptr& factor,
    const Values& linearizationPoint)
: factor_(factor->clone())
{
  // Extract keys stashed in linear factor
  BOOST_FOREACH(const Index& idx, factor_->keys())
    keys_.push_back(idx);
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
  BOOST_FOREACH(const gtsam::Key& key, keys())
    csub.insert(key, c.at(key));

  // create dummy ordering for evaluation
  OrderingOrdered ordering = *csub.orderingArbitrary();
  VectorValuesOrdered delta = linearizationPoint_->localCoordinates(csub, ordering);

  // Change keys on stored factor
  BOOST_FOREACH(gtsam::Index& index, factor_->keys())
    index = ordering[index];

  // compute error
  double error = factor_->error(delta);

  // change keys back
  factor_->keys() = keys();

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
GaussianFactorOrdered::shared_ptr LinearContainerFactor::order(const OrderingOrdered& ordering) const {
  // clone factor
  boost::shared_ptr<GaussianFactorOrdered> result = factor_->clone();

  // rekey
  BOOST_FOREACH(Index& key, result->keys())
    key = ordering[key];

  return result;
}

/* ************************************************************************* */
GaussianFactorOrdered::shared_ptr LinearContainerFactor::linearize(
    const Values& c, const OrderingOrdered& ordering) const {
  if (!hasLinearizationPoint())
    return order(ordering);

  // Extract subset of values
  Values subsetC;
  BOOST_FOREACH(const gtsam::Key& key, this->keys())
    subsetC.insert(key, c.at(key));

  // Create a temp ordering for this factor
  OrderingOrdered localOrdering = *subsetC.orderingArbitrary();

  // Determine delta between linearization points using new ordering
  VectorValuesOrdered delta = linearizationPoint_->localCoordinates(subsetC, localOrdering);

  // clone and reorder linear factor to final ordering
  GaussianFactorOrdered::shared_ptr linFactor = order(localOrdering);
  if (isJacobian()) {
    JacobianFactorOrdered::shared_ptr jacFactor = boost::dynamic_pointer_cast<JacobianFactorOrdered>(linFactor);
    jacFactor->getb() = -jacFactor->unweighted_error(delta);
  } else {
    HessianFactorOrdered::shared_ptr hesFactor = boost::dynamic_pointer_cast<HessianFactorOrdered>(linFactor);
    size_t dim = hesFactor->linearTerm().size();
    Eigen::Block<HessianFactorOrdered::Block> Gview = hesFactor->info().block(0, 0, dim, dim);
    Vector deltaVector = delta.asVector();
    Vector G_delta = Gview.selfadjointView<Eigen::Upper>() * deltaVector;
    hesFactor->constantTerm() += deltaVector.dot(G_delta) - 2.0 * deltaVector.dot(hesFactor->linearTerm());
    hesFactor->linearTerm() -= G_delta;
  }

  // reset ordering
  BOOST_FOREACH(Index& idx, linFactor->keys())
    idx = ordering[localOrdering.key(idx)];
  return linFactor;
}

/* ************************************************************************* */
bool LinearContainerFactor::isJacobian() const {
  return boost::dynamic_pointer_cast<JacobianFactorOrdered>(factor_);
}

/* ************************************************************************* */
bool LinearContainerFactor::isHessian() const {
  return boost::dynamic_pointer_cast<HessianFactorOrdered>(factor_);
}

/* ************************************************************************* */
JacobianFactorOrdered::shared_ptr LinearContainerFactor::toJacobian() const {
  return boost::dynamic_pointer_cast<JacobianFactorOrdered>(factor_);
}

/* ************************************************************************* */
HessianFactorOrdered::shared_ptr LinearContainerFactor::toHessian() const {
  return boost::dynamic_pointer_cast<HessianFactorOrdered>(factor_);
}

/* ************************************************************************* */
GaussianFactorOrdered::shared_ptr LinearContainerFactor::negate(const OrderingOrdered& ordering) const {
  GaussianFactorOrdered::shared_ptr result = factor_->negate();
  BOOST_FOREACH(Key& key, result->keys())
    key = ordering[key];
  return result;
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr LinearContainerFactor::negate() const {
  GaussianFactorOrdered::shared_ptr antifactor = factor_->negate(); // already has keys in place
  return NonlinearFactor::shared_ptr(new LinearContainerFactor(antifactor,linearizationPoint_));
}

/* ************************************************************************* */
NonlinearFactorGraph LinearContainerFactor::convertLinearGraph(
    const GaussianFactorGraphOrdered& linear_graph,  const OrderingOrdered& ordering,
    const Values& linearizationPoint) {
  NonlinearFactorGraph result;
  BOOST_FOREACH(const GaussianFactorOrdered::shared_ptr& f, linear_graph)
    if (f)
      result.push_back(NonlinearFactorGraph::sharedFactor(
          new LinearContainerFactor(f, ordering, linearizationPoint)));
  return result;
}

/* ************************************************************************* */
} // \namespace gtsam



