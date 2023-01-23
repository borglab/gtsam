/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file ReferenceFrameFactor.h
 * @brief A constraint for combining graphs by common landmarks and a transform node
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Transform function that must be specialized specific domains
 * @tparam T is a Transform type
 * @tparam P is a point type
 */
template<class T, class P>
P transform_point(
    const T& trans, const P& global,
    OptionalMatrixType Dtrans,
    OptionalMatrixType Dglobal) {
  return trans.transformFrom(global, Dtrans, Dglobal);
}

/**
 * A constraint between two landmarks in separate maps
 * Templated on:
 *   Point     : Type of landmark
 *   Transform : Transform variable class
 *
 * The transform is defined as transforming global to local:
 *   l = lTg * g
 *
 * The Point and Transform concepts must be Lie types, and the transform
 * relationship "Point = transformFrom(Transform, Point)" must exist.
 *
 * To implement this function in new domains, specialize a new version of
 * Point transform_point<Transform,Point>(transform, global, Dtrans, Dglobal)
 * to use the correct point and transform types.
 *
 * This base class should be specialized to implement the cost function for
 * specific classes of landmarks
 */
template<class POINT, class TRANSFORM>
class ReferenceFrameFactor : public NoiseModelFactorN<POINT, TRANSFORM, POINT> {
protected:
  /** default constructor for serialization only */
  ReferenceFrameFactor() {}

public:
  typedef NoiseModelFactorN<POINT, TRANSFORM, POINT> Base;
  typedef ReferenceFrameFactor<POINT, TRANSFORM> This;

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  typedef POINT Point;
  typedef TRANSFORM Transform;

  /**
   * General constructor with arbitrary noise model (constrained or otherwise)
   */
  ReferenceFrameFactor(Key globalKey, Key transKey, Key localKey, const noiseModel::Base::shared_ptr& model)
  : Base(model,globalKey, transKey, localKey) {}

  /**
   * Construct a hard frame of reference reference constraint with equal mu values for
   * each degree of freedom.
   */
  ReferenceFrameFactor(double mu, Key globalKey, Key transKey, Key localKey)
  : Base(globalKey, transKey, localKey, Point().dim(), mu) {}

  /**
   * Simple soft constraint constructor for frame of reference, with equal weighting for
   * each degree of freedom.
   */
  ReferenceFrameFactor(Key globalKey, Key transKey, Key localKey, double sigma = 1e-2)
  : Base(noiseModel::Isotropic::Sigma(traits<POINT>::dimension, sigma),
      globalKey, transKey, localKey) {}

  ~ReferenceFrameFactor() override{}

  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new This(*this))); }

  /** Combined cost and derivative function using boost::optional */
  Vector evaluateError(const Point& global, const Transform& trans, const Point& local,
        OptionalMatrixType Dforeign, OptionalMatrixType Dtrans,
        OptionalMatrixType Dlocal) const override {
    Point newlocal = transform_point<Transform,Point>(trans, global, Dtrans, Dforeign);
    if (Dlocal) {
      *Dlocal = -1* Matrix::Identity(traits<Point>::dimension, traits<Point>::dimension);
    }
    return traits<Point>::Local(local,newlocal);
  }

  void print(const std::string& s="",
      const gtsam::KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << ": ReferenceFrameFactor("
        << "Global: " << keyFormatter(this->key1()) << ","
        << " Transform: " << keyFormatter(this->key2()) << ","
        << " Local: " << keyFormatter(this->key3()) << ")\n";
    this->noiseModel_->print("  noise model");
  }

  // access - convenience functions
  Key global_key() const { return this->key1(); }
  Key transform_key() const { return this->key2(); }
  Key local_key() const { return this->key3(); }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NonlinearFactor3",
        boost::serialization::base_object<Base>(*this));
  }
};

/// traits
template<class T1, class T2>
struct traits<ReferenceFrameFactor<T1, T2> > : public Testable<ReferenceFrameFactor<T1, T2> > {};

} // \namespace gtsam
