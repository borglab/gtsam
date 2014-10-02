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

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/make_shared.hpp>

namespace gtsam {

/* ************************************************************************* */

void NonlinearFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  std::cout << s << "  keys = { ";
  BOOST_FOREACH(Key key, this->keys()) {
    std::cout << keyFormatter(key) << " ";
  }
  std::cout << "}" << std::endl;
}

bool NonlinearFactor::equals(const NonlinearFactor& f, double tol) const {
  return Base::equals(f);
}

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

NonlinearFactor::shared_ptr NonlinearFactor::rekey(
    const std::vector<Key>& new_keys) const {
  assert(new_keys.size() == this->keys().size());
  shared_ptr new_factor = clone();
  new_factor->keys() = new_keys;
  return new_factor;
}

/* ************************************************************************* */

void NoiseModelFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  this->noiseModel_->print("  noise model: ");
}

bool NoiseModelFactor::equals(const NonlinearFactor& f, double tol) const {
  const NoiseModelFactor* e = dynamic_cast<const NoiseModelFactor*>(&f);
  return e && Base::equals(f, tol)
      && ((!noiseModel_ && !e->noiseModel_)
          || (noiseModel_ && e->noiseModel_
              && noiseModel_->equals(*e->noiseModel_, tol)));
}

static void check(const SharedNoiseModel& noiseModel, const Vector& b) {
  if (!noiseModel)
    throw std::invalid_argument("NoiseModelFactor: no NoiseModel.");
  if ((size_t) b.size() != noiseModel->dim())
    throw std::invalid_argument(
        "NoiseModelFactor was created with a NoiseModel of incorrect dimension.");
}

Vector NoiseModelFactor::whitenedError(const Values& c) const {
  const Vector b = unwhitenedError(c);
  check(noiseModel_, b);
  return noiseModel_->whiten(b);
}

double NoiseModelFactor::error(const Values& c) const {
  if (this->active(c)) {
    const Vector b = unwhitenedError(c);
    check(noiseModel_, b);
    return 0.5 * noiseModel_->distance(b);
  } else {
    return 0.0;
  }
}

boost::shared_ptr<GaussianFactor> NoiseModelFactor::linearize(
    const Values& x) const {

  // Only linearize if the factor is active
  if (!this->active(x))
    return boost::shared_ptr<JacobianFactor>();

  // Call evaluate error to get Jacobians and RHS vector b
  std::vector<Matrix> A(this->size());
  Vector b = -unwhitenedError(x, A);
  check(noiseModel_, b);

  // Whiten the corresponding system now
  this->noiseModel_->WhitenSystem(A, b);

  // Fill in terms, needed to create JacobianFactor below
  std::vector<std::pair<Key, Matrix> > terms(this->size());
  for (size_t j = 0; j < this->size(); ++j) {
    terms[j].first = this->keys()[j];
    terms[j].second.swap(A[j]);
  }

  // TODO pass unwhitened + noise model to Gaussian factor
  // For now, only linearized constrained factors have noise model at linear level!!!
  noiseModel::Constrained::shared_ptr constrained = //
      boost::dynamic_pointer_cast<noiseModel::Constrained>(this->noiseModel_);
  if (constrained) {
    // Create a factor of reduced row dimension d_
    size_t d_ = terms[0].second.rows() - constrained->dim();
    Vector zero_ = zero(d_);
    Vector b_ = concatVectors(2, &b, &zero_);
    noiseModel::Constrained::shared_ptr model = constrained->unit(d_);
    return boost::make_shared<JacobianFactor>(terms, b_, model);
  } else
    return GaussianFactor::shared_ptr(new JacobianFactor(terms, b));
}

/* ************************************************************************* */

} // \namespace gtsam
