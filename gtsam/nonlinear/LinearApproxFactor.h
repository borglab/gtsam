/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LinearApproxFactor.h
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/SharedDiagonal.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <vector>

namespace gtsam{

/**
 * A dummy factor that takes a linearized factor and inserts it into
 * a nonlinear graph.  This version uses exactly one type of variable.
 *
 * IMPORTANT: Don't use this factor - used LinearizedFactor instead
 */
template <class VALUES, class KEY>
class LinearApproxFactor : public NonlinearFactor<VALUES> {

public:
  /** type for the variable */
  typedef typename KEY::Value X;

  /** base type */
  typedef NonlinearFactor<VALUES> Base;

  /** shared pointer for convenience */
  typedef boost::shared_ptr<LinearApproxFactor<VALUES,KEY> > shared_ptr;

  /** typedefs for key vectors */
  typedef std::vector<KEY> KeyVector;

protected:
  /** hold onto the factor itself */
  // store components of a jacobian factor
  typedef std::map<KEY, Matrix> KeyMatrixMap;
  KeyMatrixMap matrices_;
  Vector b_;
  SharedDiagonal model_; /// separate from the noisemodel in NonlinearFactor due to Diagonal/Gaussian

  /** linearization points for error calculation */
  VALUES lin_points_;

  /** keep keys for the factor */
  KeyVector nonlinearKeys_;

  /** default constructor for serialization */
  LinearApproxFactor() {}

  /**
   * use this for derived classes with keys that don't copy easily
   */
  LinearApproxFactor(size_t dim, const VALUES& lin_points)
  : Base(noiseModel::Unit::Create(dim)), lin_points_(lin_points) {}
  LinearApproxFactor(SharedDiagonal model)
  : Base(model), model_(model) {}
  LinearApproxFactor(SharedDiagonal model, const VALUES& lin_points)
  : Base(model), model_(model), lin_points_(lin_points) {}

public:

  virtual ~LinearApproxFactor() {}

  /** Vector of errors, unwhitened ! */
  virtual Vector unwhitenedError(const VALUES& c) const;

  /**
   * linearize to a GaussianFactor
   * Reconstructs the linear factor from components to ensure that
   * the ordering is correct
   */
  virtual boost::shared_ptr<GaussianFactor> linearize(
      const VALUES& c, const Ordering& ordering) const;

  /**
   * Create a symbolic factor using the given ordering to determine the
   * variable indices.
   */
  IndexFactor::shared_ptr symbolic(const Ordering& ordering) const;

  /** get access to nonlinear keys */
  KeyVector nonlinearKeys() const { return nonlinearKeys_; }

  /** override print function */
  virtual void print(const std::string& s="") const;

  /** access to b vector of gaussian */
  Vector get_b() const { return b_; }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NonlinearFactor",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(matrices_);
    ar & BOOST_SERIALIZATION_NVP(b_);
    ar & BOOST_SERIALIZATION_NVP(model_);
    ar & BOOST_SERIALIZATION_NVP(lin_points_);
    ar & BOOST_SERIALIZATION_NVP(nonlinearKeys_);
  }
};

} // \namespace gtsam
