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
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * We override this method to provide
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if (H) {
      // H should be pre-allocated
      assert(H->size()==size());

      // Create and zero out blocks to be passed to expression_
      JacobianMap blocks;
      blocks.reserve(size());
      for (DenseIndex i = 0; i < size(); i++) {
        Matrix& Hi = H->at(i);
        Hi.resize(Dim, dimensions_[i]);
        Hi.setZero(); // zero out
        Eigen::Block<Matrix> block = Hi.block(0, 0, Dim, dimensions_[i]);
        blocks.push_back(std::make_pair(keys_[i], block));
      }

      T value = expression_.value(x, blocks);
      return measurement_.localCoordinates(value);
    } else {
      const T& value = expression_.value(x);
      return measurement_.localCoordinates(value);
    }
  }

  virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x) const {

    // This method has been heavily optimized for maximum performance.
    // We allocate a VerticalBlockMatrix on the stack first, and then create
    // Eigen::Block<Matrix> views on this piece of memory which is then passed
    // to [expression_.value] below, which writes directly into Ab_.

    // Another malloc saved by creating a Matrix on the stack
    double memory[Dim * augmentedCols_];
    Eigen::Map<Eigen::Matrix<double, Dim, Eigen::Dynamic> > //
    matrix(memory, Dim, augmentedCols_);
    matrix.setZero(); // zero out

    // Construct block matrix, is of right size but un-initialized
    VerticalBlockMatrix Ab(dimensions_, matrix, true);

    // Create blocks into Ab_ to be passed to expression_
    JacobianMap blocks;
    blocks.reserve(size());
    for (DenseIndex i = 0; i < size(); i++)
      blocks.push_back(std::make_pair(keys_[i], Ab(i)));

    // Evaluate error to get Jacobians and RHS vector b
    T value = expression_.value(x, blocks); // <<< Reverse AD happens here !
    Ab(size()).col(0) = -measurement_.localCoordinates(value);

    // Whiten the corresponding system now
    // TODO ! this->noiseModel_->WhitenSystem(Ab);

    // TODO pass unwhitened + noise model to Gaussian factor
    // For now, only linearized constrained factors have noise model at linear level!!!
    noiseModel::Constrained::shared_ptr constrained = //
        boost::dynamic_pointer_cast<noiseModel::Constrained>(this->noiseModel_);
    if (constrained) {
      return boost::make_shared<JacobianFactor>(this->keys(), Ab,
          constrained->unit());
    } else
      return boost::make_shared<JacobianFactor>(this->keys(), Ab);
  }
};
// ExpressionFactor

}

