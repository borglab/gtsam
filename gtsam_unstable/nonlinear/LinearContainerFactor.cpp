/**
 * @file LinearContainerFactor.cpp
 *
 * @date Jul 6, 2012
 * @author Alex Cunningham
 */

#include <gtsam_unstable/nonlinear/LinearContainerFactor.h>

#include <boost/foreach.hpp>

namespace gtsam {

/* ************************************************************************* */
void LinearContainerFactor::rekeyFactor(const Ordering& ordering) {
	Ordering::InvertedMap invOrdering = ordering.invert(); // TODO: inefficient - make more selective ordering invert
	rekeyFactor(invOrdering);
}

/* ************************************************************************* */
void LinearContainerFactor::rekeyFactor(const Ordering::InvertedMap& invOrdering) {
	BOOST_FOREACH(Index& idx, factor_->keys()) {
		Key fullKey = invOrdering.find(idx)->second;
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
LinearContainerFactor::LinearContainerFactor(const GaussianFactor::shared_ptr& factor,
		const boost::optional<Values>& linearizationPoint)
: factor_(factor), linearizationPoint_(linearizationPoint) {
	// Extract keys stashed in linear factor
	BOOST_FOREACH(const Index& idx, factor_->keys())
		keys_.push_back(idx);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const JacobianFactor& factor, const Ordering& ordering,
		const Values& linearizationPoint)
: factor_(factor.clone()) {
	rekeyFactor(ordering);
	initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const HessianFactor& factor, const Ordering& ordering,
		const Values& linearizationPoint)
: factor_(factor.clone()) {
	rekeyFactor(ordering);
	initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const GaussianFactor::shared_ptr& factor, const Ordering& ordering,
		const Values& linearizationPoint)
: factor_(factor->clone()) {
	rekeyFactor(ordering);
	initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const GaussianFactor::shared_ptr& factor,
		const Values& linearizationPoint)
: factor_(factor->clone())
{
	// Extract keys stashed in linear factor
	BOOST_FOREACH(const Index& idx, factor_->keys())
		keys_.push_back(idx);
	initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(const JacobianFactor& factor,
		const Ordering::InvertedMap& inverted_ordering,
		const Values& linearizationPoint)
: factor_(factor.clone()) {
	rekeyFactor(inverted_ordering);
	initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(const HessianFactor& factor,
		const Ordering::InvertedMap& inverted_ordering,
		const Values& linearizationPoint)
: factor_(factor.clone()) {
	rekeyFactor(inverted_ordering);
	initializeLinearizationPoint(linearizationPoint);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const GaussianFactor::shared_ptr& factor,
		const Ordering::InvertedMap& ordering,
		const Values& linearizationPoint)
: factor_(factor->clone()) {
	rekeyFactor(ordering);
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

	// Extract subset of values for comparision
	Values csub;
	BOOST_FOREACH(const gtsam::Key& key, keys())
		csub.insert(key, c.at(key));

	// create dummy ordering for evaluation
	Ordering ordering = *csub.orderingArbitrary();
	VectorValues delta = linearizationPoint_->localCoordinates(csub, ordering);

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
GaussianFactor::shared_ptr LinearContainerFactor::order(const Ordering& ordering) const {
	// clone factor
	boost::shared_ptr<GaussianFactor> result = factor_->clone();

	// rekey
	BOOST_FOREACH(Index& key, result->keys())
		key = ordering[key];

	return result;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr LinearContainerFactor::linearize(
		const Values& c, const Ordering& ordering) const {
	return order(ordering);
}

/* ************************************************************************* */
bool LinearContainerFactor::isJacobian() const {
	return boost::shared_dynamic_cast<JacobianFactor>(factor_);
}

/* ************************************************************************* */
JacobianFactor::shared_ptr LinearContainerFactor::toJacobian() const {
	return boost::shared_dynamic_cast<JacobianFactor>(factor_);
}

/* ************************************************************************* */
HessianFactor::shared_ptr LinearContainerFactor::toHessian() const {
	return boost::shared_dynamic_cast<HessianFactor>(factor_);
}

/* ************************************************************************* */
GaussianFactor::shared_ptr LinearContainerFactor::negate(const Ordering& ordering) const {
	GaussianFactor::shared_ptr result = factor_->negate();
	BOOST_FOREACH(Key& key, result->keys())
		key = ordering[key];
	return result;
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr LinearContainerFactor::negate() const {
	GaussianFactor::shared_ptr antifactor = factor_->negate(); // already has keys in place
	return NonlinearFactor::shared_ptr(new LinearContainerFactor(antifactor,linearizationPoint_));
}

/* ************************************************************************* */
NonlinearFactorGraph LinearContainerFactor::convertLinearGraph(
		const GaussianFactorGraph& linear_graph,	const Ordering& ordering) {
	return convertLinearGraph(linear_graph, ordering.invert());
}

/* ************************************************************************* */
NonlinearFactorGraph LinearContainerFactor::convertLinearGraph(
		const GaussianFactorGraph& linear_graph,	const InvertedOrdering& invOrdering) {
	NonlinearFactorGraph result;
	BOOST_FOREACH(const GaussianFactor::shared_ptr& f, linear_graph)
		if (f)
			result.push_back(NonlinearFactorGraph::sharedFactor(new LinearContainerFactor(f, invOrdering)));
	return result;
}

/* ************************************************************************* */
} // \namespace gtsam



