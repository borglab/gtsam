/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactor.cpp
 * @brief   Nonlinear Factor base classes
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>

namespace gtsam {

/* ************************************************************************* */
const Values& GetValues(const HybridValues& c) {
  return c.nonlinear();
}

/* ************************************************************************* */
void NonlinearFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  std::cout << s << "  keys = { ";
  for(Key key: keys()) {
    std::cout << keyFormatter(key) << " ";
  }
  std::cout << "}" << std::endl;
}

/* ************************************************************************* */
bool NonlinearFactor::equals(const NonlinearFactor& f, double tol) const {
  return Base::equals(f);
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr NonlinearFactor::rekey(
    const std::map<Key, Key>& rekey_mapping) const {
  shared_ptr new_factor = clone();
  for (size_t i = 0; i < new_factor->size(); ++i) {
    Key& cur_key = new_factor->keys()[i];
    std::map<Key, Key>::const_iterator mapping = rekey_mapping.find(cur_key);
    if (mapping != rekey_mapping.end())
      cur_key = mapping->second;
  }
  return new_factor;
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr NonlinearFactor::rekey(
    const KeyVector& new_keys) const {
  assert(new_keys.size() == keys().size());
  shared_ptr new_factor = clone();
  new_factor->keys() = new_keys;
  return new_factor;
}

/* ************************************************************************* */
void NoiseModelFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  if (noiseModel_)
    noiseModel_->print("  noise model: ");
}

/* ************************************************************************* */
bool NoiseModelFactor::equals(const NonlinearFactor& f, double tol) const {
  const NoiseModelFactor* e = dynamic_cast<const NoiseModelFactor*>(&f);
  return e && Base::equals(f, tol)
      && ((!noiseModel_ && !e->noiseModel_)
          || (noiseModel_ && e->noiseModel_
              && noiseModel_->equals(*e->noiseModel_, tol)));
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr NoiseModelFactor::cloneWithNewNoiseModel(
    const SharedNoiseModel newNoise) const {
  NoiseModelFactor::shared_ptr new_factor = boost::dynamic_pointer_cast<NoiseModelFactor>(clone());
  new_factor->noiseModel_ = newNoise;
  return new_factor;
}

/* ************************************************************************* */
static void check(const SharedNoiseModel& noiseModel, size_t m) {
  if (noiseModel && m != noiseModel->dim())
    throw std::invalid_argument(
        boost::str(
            boost::format(
                "NoiseModelFactor: NoiseModel has dimension %1% instead of %2%.")
                % noiseModel->dim() % m));
}

/* ************************************************************************* */
Vector NoiseModelFactor::whitenedError(const Values& c) const {
  const Vector b = unwhitenedError(c);
  check(noiseModel_, b.size());
  return noiseModel_ ? noiseModel_->whiten(b) : b;
}

/* ************************************************************************* */
Vector NoiseModelFactor::unweightedWhitenedError(const Values& c) const {
  const Vector b = unwhitenedError(c);
  check(noiseModel_, b.size());
  return noiseModel_ ? noiseModel_->unweightedWhiten(b) : b;
}

/* ************************************************************************* */
double NoiseModelFactor::weight(const Values& c) const {
  if (active(c)) {
    if (noiseModel_) {
      const Vector b = unwhitenedError(c);
      check(noiseModel_, b.size());
      return noiseModel_->weight(b);
    }
    else
      return 1.0;
  } else {
    return 0.0;
  }
}

/* ************************************************************************* */
double NoiseModelFactor::error(const Values& c) const {
  if (active(c)) {
    const Vector b = unwhitenedError(c);
    check(noiseModel_, b.size());
    if (noiseModel_)
      return noiseModel_->loss(noiseModel_->squaredMahalanobisDistance(b));
    else
      return 0.5 * b.squaredNorm();
  } else {
    return 0.0;
  }
}

/* ************************************************************************* */
boost::shared_ptr<GaussianFactor> NoiseModelFactor::linearize(
    const Values& x) const {

  // Only linearize if the factor is active
  if (!active(x))
    return boost::shared_ptr<JacobianFactor>();

  // Call evaluate error to get Jacobians and RHS vector b
  std::vector<Matrix> A(size());
  Vector b = -unwhitenedError(x, A);
  check(noiseModel_, b.size());

  // Whiten the corresponding system now
  if (noiseModel_)
    noiseModel_->WhitenSystem(A, b);

  // Fill in terms, needed to create JacobianFactor below
  std::vector<std::pair<Key, Matrix> > terms(size());
  for (size_t j = 0; j < size(); ++j) {
    terms[j].first = keys()[j];
    terms[j].second.swap(A[j]);
  }

  // TODO pass unwhitened + noise model to Gaussian factor
  using noiseModel::Constrained;
  if (noiseModel_ && noiseModel_->isConstrained())
    return GaussianFactor::shared_ptr(
        new JacobianFactor(terms, b,
            boost::static_pointer_cast<Constrained>(noiseModel_)->unit()));
  else {
    return GaussianFactor::shared_ptr(new JacobianFactor(terms, b));
  }
}

/* ************************************************************************* */

} // \namespace gtsam
