/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Expression.h
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief Expressions for Block Automatic Differentiation
 */

#include <gtsam_unstable/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm.hpp>
#include <numeric>

class ExpressionFactorWriteableJacobianFactorTest;

namespace gtsam {

/**
 * Special version of JacobianFactor that allows Jacobians to be written
 * Eliminates a large proportion of overhead
 * Note all ExpressionFactor<T> are friends, not for general consumption.
 */
class WriteableJacobianFactor: public JacobianFactor {

public:

  /**
   *  Constructor
   *  @param keys in some order
   *  @param diemnsions of the variables in same order
   *  @param m output dimension
   *  @param model noise model (default NULL)
   */
  template<class KEYS, class DIMENSIONS>
  WriteableJacobianFactor(const KEYS& keys, const DIMENSIONS& dims,
      DenseIndex m, const SharedDiagonal& model = SharedDiagonal()) {

    // Check noise model dimension
    if (model && (DenseIndex) model->dim() != m)
      throw InvalidNoiseModel(m, model->dim());

    // copy the keys
    keys_.resize(keys.size());
    std::copy(keys.begin(), keys.end(), keys_.begin());

    // Check number of variables
    if (dims.size() != keys_.size())
      throw std::invalid_argument(
          "WriteableJacobianFactor: size of dimensions and keys do not agree.");

    Ab_ = VerticalBlockMatrix(dims.begin(), dims.end(), m, true);
    Ab_.matrix().setZero();
    model_ = model;
  }

  VerticalBlockMatrix& Ab() {
    return Ab_;
  }

//  friend class ::ExpressionFactorWriteableJacobianFactorTest;
//  template<typename T>
//  friend class ExpressionFactor;
};

/**
 * Factor that supports arbitrary expressions via AD
 */
template<class T>
class ExpressionFactor: public NoiseModelFactor {

  T measurement_; ///< the measurement to be compared with the expression
  Expression<T> expression_; ///< the expression that is AD enabled
  std::vector<size_t> dimensions_; ///< dimensions of the Jacobian matrices
  size_t augmentedCols_; ///< total number of columns + 1 (for RHS)

  static const int Dim = traits::dimension<T>::value;

public:

  /// Constructor
  ExpressionFactor(const SharedNoiseModel& noiseModel, //
      const T& measurement, const Expression<T>& expression) :
      measurement_(measurement), expression_(expression) {
    if (!noiseModel)
      throw std::invalid_argument("ExpressionFactor: no NoiseModel.");
    if (noiseModel->dim() != Dim)
      throw std::invalid_argument(
          "ExpressionFactor was created with a NoiseModel of incorrect dimension.");
    noiseModel_ = noiseModel;

    // Get dimensions of Jacobian matrices
    // An Expression is assumed unmutable, so we do this now
    std::map<Key, size_t> map;
    expression_.dims(map);
    size_t n = map.size();

    keys_.resize(n);
    boost::copy(map | boost::adaptors::map_keys, keys_.begin());

    dimensions_.resize(n);
    boost::copy(map | boost::adaptors::map_values, dimensions_.begin());

    // Add sizes to know how much memory to allocate on stack in linearize
    augmentedCols_ = std::accumulate(dimensions_.begin(), dimensions_.end(), 1);

#ifdef DEBUG_ExpressionFactor
    BOOST_FOREACH(size_t d, dimensions_)
    std::cout << d << " ";
    std::cout << " -> " << Dim << "x" << augmentedCols_ << std::endl;
#endif
  }

  /**
   * Error function *without* the NoiseModel, \f$ h(x)-z \f$.
   * We override this method to provide
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if (H) {
      // H should be pre-allocated
      assert(H->size()==size());

      VerticalBlockMatrix Ab(dimensions_, Dim);

      // Wrap keys and VerticalBlockMatrix into structure passed to expression_
      JacobianMap map(keys_, Ab);
      Ab.matrix().setZero();

      // Evaluate error to get Jacobians and RHS vector b
      T value = expression_.value(x, map); // <<< Reverse AD happens here !

      // Copy blocks into the vector of jacobians passed in
      for (DenseIndex i = 0; i < size(); i++)
        H->at(i) = Ab(i);

      return measurement_.localCoordinates(value);
    } else {
      const T& value = expression_.value(x);
      return measurement_.localCoordinates(value);
    }
  }

  virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x) const {

    // Create noise model
    SharedDiagonal model;
    noiseModel::Constrained::shared_ptr constrained = //
        boost::dynamic_pointer_cast<noiseModel::Constrained>(this->noiseModel_);
    if (constrained)
      model = constrained->unit();

    // Create a writeable JacobianFactor in advance
    boost::shared_ptr<WriteableJacobianFactor> factor = boost::make_shared<
        WriteableJacobianFactor>(keys_, dimensions_,
        traits::dimension<T>::value, model);

    // Wrap keys and VerticalBlockMatrix into structure passed to expression_
    JacobianMap map(keys_, factor->Ab());

    // Evaluate error to get Jacobians and RHS vector b
    T value = expression_.value(x, map); // <<< Reverse AD happens here !
    factor->Ab()(size()).col(0) = -measurement_.localCoordinates(value);

    // Whiten the corresponding system now
    // TODO ! this->noiseModel_->WhitenSystem(Ab);

    return factor;
  }
};
// ExpressionFactor

}

