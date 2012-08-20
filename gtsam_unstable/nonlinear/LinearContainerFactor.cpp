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
LinearContainerFactor::LinearContainerFactor(
		const JacobianFactor& factor, const Ordering& ordering)
: factor_(factor.clone()) {
	rekeyFactor(ordering);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const HessianFactor& factor, const Ordering& ordering)
: factor_(factor.clone()) {
	rekeyFactor(ordering);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const GaussianFactor::shared_ptr& factor, const Ordering& ordering)
: factor_(factor->clone()) {
	rekeyFactor(ordering);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const GaussianFactor::shared_ptr& factor)
: factor_(factor->clone())
{
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(const JacobianFactor& factor,
		const Ordering::InvertedMap& inverted_ordering)
: factor_(factor.clone()) {
	rekeyFactor(inverted_ordering);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(const HessianFactor& factor,
		const Ordering::InvertedMap& inverted_ordering)
: factor_(factor.clone()) {
	rekeyFactor(inverted_ordering);
}

/* ************************************************************************* */
LinearContainerFactor::LinearContainerFactor(
		const GaussianFactor::shared_ptr& factor,
		const Ordering::InvertedMap& ordering)
: factor_(factor->clone()) {
	rekeyFactor(ordering);
}

/* ************************************************************************* */
void LinearContainerFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
	Base::print(s+"LinearContainerFactor", keyFormatter);
	if (factor_)
		factor_->print("   Stored Factor", keyFormatter);
}

/* ************************************************************************* */
bool LinearContainerFactor::equals(const NonlinearFactor& f, double tol) const {
	const LinearContainerFactor* jcf = dynamic_cast<const LinearContainerFactor*>(&f);
	return jcf && factor_->equals(*jcf->factor_, tol) && NonlinearFactor::equals(f);
}

/* ************************************************************************* */
double LinearContainerFactor::error(const Values& c) const {
	//	VectorValues vecvalues;
	//	// FIXME: add values correctly here
	//	return factor_.error(vecvalues);
		return 0; // FIXME: placeholder
}

/* ************************************************************************* */
size_t LinearContainerFactor::dim() const {
	if (isJacobian())
		return toJacobian()->get_model()->dim();
	else
		return 1; // Hessians don't have true dimension
}

/* ************************************************************************* */
boost::shared_ptr<GaussianFactor>
LinearContainerFactor::linearize(const Values& c, const Ordering& ordering) const {
	// clone factor
	boost::shared_ptr<GaussianFactor> result = factor_->clone();

	// rekey
	BOOST_FOREACH(Index& key, result->keys())
		key = ordering[key];

	return result;
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
	return NonlinearFactor::shared_ptr(new LinearContainerFactor(antifactor));
}

/* ************************************************************************* */
} // \namespace gtsam



