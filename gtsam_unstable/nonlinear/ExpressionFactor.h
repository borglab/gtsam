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

namespace gtsam {

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

    // Only linearize if the factor is active
    if (!active(x))
      return boost::shared_ptr<JacobianFactor>();

    // Create a writeable JacobianFactor in advance
    // In case noise model is constrained, we need to provide a noise model
    bool constrained = noiseModel_->is_constrained();
    boost::shared_ptr<JacobianFactor> factor(
        constrained ? new JacobianFactor(keys_, dimensions_, Dim,
            boost::static_pointer_cast<noiseModel::Constrained>(noiseModel_)->unit()) :
            new JacobianFactor(keys_, dimensions_, Dim));

    // Wrap keys and VerticalBlockMatrix into structure passed to expression_
    VerticalBlockMatrix& Ab = factor->matrixObject();
    JacobianMap map(keys_, Ab);

    // Zero out Jacobian so we can simply add to it
    Ab.matrix().setZero();

    // Evaluate error to get Jacobians and RHS vector b
    T value = expression_.value(x, map); // <<< Reverse AD happens here !
    Ab(size()).col(0) = -measurement_.localCoordinates(value);

    // Whiten the corresponding system, Ab already contains RHS
    Vector dummy(Dim);
    noiseModel_->WhitenSystem(Ab.matrix(),dummy);

    return factor;
  }
};
// ExpressionFactor

}

