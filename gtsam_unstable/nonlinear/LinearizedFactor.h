/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LinearizedFactor.h
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#pragma once

#include <vector>
#include <gtsam_unstable/dllexport.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>

namespace gtsam {

/**
 * A base factor class for the Jacobian and Hessian linearized factors
 */
class GTSAM_UNSTABLE_EXPORT LinearizedGaussianFactor : public NonlinearFactor {
public:
  /** base type */
  typedef NonlinearFactor Base;
  typedef LinearizedGaussianFactor This;

  /** shared pointer for convenience */
  typedef std::shared_ptr<LinearizedGaussianFactor> shared_ptr;

protected:

  /** linearization points for error calculation */
  Values lin_points_;

public:

  /** default constructor for serialization */
  LinearizedGaussianFactor() {};

  /**
   * @param gaussian:   A jacobian or hessian factor
   * @param lin_points: The linearization points for, at least, the variables used by this factor
   */
  LinearizedGaussianFactor(const GaussianFactor::shared_ptr& gaussian, const Values& lin_points);

  ~LinearizedGaussianFactor() override {};

  // access functions
  const Values& linearizationPoint() const { return lin_points_; }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("LinearizedGaussianFactor",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(lin_points_);
  }

};

/**
 * A factor that takes a linear, Jacobian factor and inserts it into
 * a nonlinear graph.
 */
class GTSAM_UNSTABLE_EXPORT LinearizedJacobianFactor : public LinearizedGaussianFactor {

public:
  /** base type */
  typedef LinearizedGaussianFactor Base;
  typedef LinearizedJacobianFactor This;

  /** shared pointer for convenience */
  typedef std::shared_ptr<LinearizedJacobianFactor> shared_ptr;

  typedef VerticalBlockMatrix::Block ABlock;
  typedef VerticalBlockMatrix::constBlock constABlock;
  typedef VerticalBlockMatrix::Block::ColXpr BVector;
  typedef VerticalBlockMatrix::constBlock::ConstColXpr constBVector;

protected:

//  // store components of a jacobian factor
//  typedef std::map<Key, Matrix> KeyMatrixMap;
//  KeyMatrixMap matrices_;
//  Vector b_;

  VerticalBlockMatrix Ab_;      // the block view of the full matrix

public:

  /** default constructor for serialization */
  LinearizedJacobianFactor();

  /**
   * @param jacobian:   A jacobian factor
   * @param lin_points: The linearization points for, at least, the variables used by this factor
   */
  LinearizedJacobianFactor(const JacobianFactor::shared_ptr& jacobian, const Values& lin_points);

  ~LinearizedJacobianFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  // Testable

  /** print function */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** equals function with optional tolerance */
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  // access functions
  const constBVector b() const { return Ab_(size()).col(0); }
  const constABlock A() const { return Ab_.range(0, size()); };
  const constABlock A(Key key) const { return Ab_(std::find(begin(), end(), key) - begin()); }

  /** get the dimension of the factor (number of rows on linearization) */
  size_t dim() const override { return Ab_.rows(); };

  /** Calculate the error of the factor */
  double error(const Values& c) const override;

  /**
   * linearize to a GaussianFactor
   * Reimplemented from NoiseModelFactor to directly copy out the
   * matrices and only update the RHS b with an updated residual
   */
  std::shared_ptr<GaussianFactor> linearize(const Values& c) const override;

  /** (A*x-b) */
  Vector error_vector(const Values& c) const;

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("LinearizedJacobianFactor",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(Ab_);
  }
};

/// traits
template<>
struct traits<LinearizedJacobianFactor> : public Testable<LinearizedJacobianFactor> {
};

/**
 * A factor that takes a linear, Hessian factor and inserts it into
 * a nonlinear graph.
 */
class GTSAM_UNSTABLE_EXPORT LinearizedHessianFactor : public LinearizedGaussianFactor {

public:
  /** base type */
  typedef LinearizedGaussianFactor Base;
  typedef LinearizedHessianFactor This;

  /** shared pointer for convenience */
  typedef std::shared_ptr<LinearizedHessianFactor> shared_ptr;

  /** hessian block data types */
  typedef SymmetricBlockMatrix::Block Block; ///< A block from the Hessian matrix
  typedef SymmetricBlockMatrix::constBlock constBlock; ///< A block from the Hessian matrix (const version)

  typedef SymmetricBlockMatrix::Block::ColXpr Column; ///< A column containing the linear term h
  typedef SymmetricBlockMatrix::constBlock::ColXpr constColumn; ///< A column containing the linear term h (const version)

protected:

  SymmetricBlockMatrix info_; ///< The block view of the full information matrix, s.t. the quadratic
                              ///  error is 0.5*[x -1]'*H*[x -1]

public:

  /** default constructor for serialization */
  LinearizedHessianFactor();

  /**
   * Use this constructor with the ordering used to linearize the graph
   * @param hessian:    A hessian factor
   * @param lin_points: The linearization points for, at least, the variables used by this factor
   */
  LinearizedHessianFactor(const HessianFactor::shared_ptr& hessian, const Values& lin_points);

  ~LinearizedHessianFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  // Testable

  /** print function */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** equals function with optional tolerance */
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /** Return the constant term \f$ f \f$ as described above
   * @return The constant term \f$ f \f$
   */
  double constantTerm() const {
    const auto block = info_.diagonalBlock(size());
    return block(0, 0);
  }

  /** Return the part of linear term \f$ g \f$ as described above corresponding to the requested variable.
   * @param j Which block row to get, as an iterator pointing to the slot in this factor.  You can
   * use, for example, begin() + 2 to get the 3rd variable in this factor.
   * @return The linear term \f$ g \f$ */
  constColumn linearTerm(const_iterator j) const {
    return info_.aboveDiagonalRange(j - begin(), size(), size(), size() + 1).col(0);
  }

  /** Return the complete linear term \f$ g \f$ as described above.
   * @return The linear term \f$ g \f$ */
  constColumn linearTerm() const {
    return info_.aboveDiagonalRange(0, size(), size(), size() + 1).col(0);
  }

  /** Return a copy of the block at (j1,j2) of the <em>upper-triangular part</em> of the
   * squared term \f$ H \f$, no data is copied.  See HessianFactor class documentation
   * above to explain that only the upper-triangular part of the information matrix is stored
   * and returned by this function.
   * @param j1 Which block row to get, as an iterator pointing to the slot in this factor.  You can
   * use, for example, begin() + 2 to get the 3rd variable in this factor.
   * @param j2 Which block column to get, as an iterator pointing to the slot in this factor.  You can
   * use, for example, begin() + 2 to get the 3rd variable in this factor.
   * @return A copy of the requested block.
   */
  Matrix squaredTerm(const_iterator j1, const_iterator j2) const {
    const DenseIndex J1 = j1 - begin();
    const DenseIndex J2 = j2 - begin();
    return info_.block(J1, J2);
  }

  /** Return the <em>upper-triangular part</em> of the full squared term, as described above.
   * See HessianFactor class documentation above to explain that only the
   * upper-triangular part of the information matrix is stored and returned by this function.
   */
  Eigen::SelfAdjointView<constBlock, Eigen::Upper> squaredTerm() const {
    return info_.selfadjointView(0, size());
  }

  /** get the dimension of the factor (number of rows on linearization) */
  size_t dim() const override { return info_.rows() - 1; }

  /** Calculate the error of the factor */
  double error(const Values& c) const override;

  /**
   * linearize to a GaussianFactor
   * Reimplemented from NoiseModelFactor to directly copy out the
   * matrices and only update the RHS b with an updated residual
   */
  std::shared_ptr<GaussianFactor> linearize(const Values& c) const override;

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("LinearizedHessianFactor",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(info_);
  }
};

/// traits
template<>
struct traits<LinearizedHessianFactor> : public Testable<LinearizedHessianFactor> {
};

} // \namespace aspn
