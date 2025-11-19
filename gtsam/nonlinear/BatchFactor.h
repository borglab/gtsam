/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchFactor.h
 * @brief   A batch of factors that linearizes to a single JacobianFactor
 * @author  Frank Dellaert
 * @author  Fan Jiang
 * @date    Nov 2025
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/StdVector>
#include <algorithm>
#include <cassert>
#include <map>
#include <type_traits>
#include <vector>

namespace gtsam {

namespace detail {

/**
 * @brief Helper to construct a factor by trying common signature patterns.
 *
 * Tries the following constructor signatures for FactorType:
 * 1. (Key1, Key2, Measurement, Model, Args...)  [Standard]
 * 2. (Measurement, Model, Key1, Key2, Args...)  [Projection/SFM]
 * 3. (Key1, Key2, Measurement, Args..., Model)  [MagFactor style]
 */
template <typename FactorType, typename K1, typename K2, typename Meas,
          typename Model, typename... Args>
static FactorType createFactor(K1 k1, K2 k2, const Meas& z, const Model& model,
                               Args&&... args) {
  if constexpr (std::is_constructible_v<FactorType, K1, K2, Meas, Model,
                                        Args...>) {
    return FactorType(k1, k2, z, model, std::forward<Args>(args)...);
  } else if constexpr (std::is_constructible_v<FactorType, Meas, Model, K1, K2,
                                               Args...>) {
    return FactorType(z, model, k1, k2, std::forward<Args>(args)...);
  } else if constexpr (std::is_constructible_v<FactorType, K1, K2, Meas,
                                               Args..., Model>) {
    return FactorType(k1, k2, z, std::forward<Args>(args)..., model);
  } else {
    // This static_assert will trigger if none of the above match.
    // We repeat the check to produce a readable error message.
    static_assert(
        std::is_constructible_v<FactorType, K1, K2, Meas, Model, Args...>,
        "BatchFactor: Could not find a matching constructor for FactorType. "
        "Tried: (K1, K2, Z, Model, Args...), (Z, Model, K1, K2, Args...), (K1, "
        "K2, Z, Args..., Model)");
    return FactorType(k1, k2, z, model, std::forward<Args>(args)...);
  }
}

}  // namespace detail

/**
 * BatchFactor is a NonlinearFactor that wraps a collection of identical
 * factors. It linearizes them all at once into a single JacobianFactor.
 *
 * This is useful for optimizing Structure-from-Motion (SfM) and SLAM graphs
 * where we have many factors of the same type (e.g., projection factors) that
 * can be grouped together to reduce overhead.
 *
 * Usage Example:
 *
 * // Assume we have GenericProjectionFactor<Pose3, Point3>
 * using ProjectionFactor = GenericProjectionFactor<Pose3, Point3>;
 *
 * // Create a batch factor
 * std::vector<Key> poses = {Symbol('x', 1)};
 * std::vector<Key> points;
 * std::vector<Point2> measurements;
 * for (int i = 0; i < 100; ++i) {
 *   points.push_back(Symbol('l', i));
 *   measurements.push_back(Point2(10, 10)); // Dummy measurement
 * }
 *
 * auto noise = noiseModel::Isotropic::Sigma(2, 1.0);
 *
 * // Construct using the helper (1 camera, 100 points)
 * auto batch = std::make_shared<BatchFactor<ProjectionFactor, 2>>(
 *     poses, points, measurements, noise);
 *
 * // Add to graph
 * NonlinearFactorGraph graph;
 * graph.add(batch);
 *
 * // Optimize as usual
 * LevenbergMarquardtOptimizer optimizer(graph, initial_values);
 * Values result = optimizer.optimize();
 *
 * @tparam FactorType The type of the individual factors
 * @tparam ErrorDim The dimension of the error vector for a single factor
 */
template <typename FactorType, int ErrorDim>
class BatchFactor : public NonlinearFactor {
 public:
  // Static assertion to ensure FactorType derives from NoiseModelFactor
  static_assert(std::is_base_of<NoiseModelFactor, FactorType>::value,
                "FactorType must derive from NoiseModelFactor");

  using Base = NonlinearFactor;
  using This = BatchFactor<FactorType, ErrorDim>;
  using shared_ptr = std::shared_ptr<This>;

 private:
  using Allocator = Eigen::aligned_allocator<FactorType>;
  std::vector<FactorType, Allocator> factors_;  ///< Contiguous storage
  struct KeyInfo {
    Key key;
    size_t dim;
    size_t slot;
    DenseIndex offset;
  };
  std::vector<KeyInfo> keyInfo_;
  std::vector<std::array<DenseIndex, FactorType::N>> indices_;
  bool useHessianFactor_{false};

 public:
  /// @name Constructors
  /// @{

  /** Default constructor */
  BatchFactor() = default;

  /** Constructor from a vector of factors (moves the vector) */
  explicit BatchFactor(std::vector<FactorType, Allocator>&& factors)
      : factors_(std::move(factors)) {
    updateKeys();
  }

  /** Constructor from a vector of factors (copies the vector) */
  explicit BatchFactor(const std::vector<FactorType, Allocator>& factors)
      : factors_(factors) {
    updateKeys();
  }

  /** Constructor from a standard vector of factors (copies the vector) */
  explicit BatchFactor(const std::vector<FactorType>& factors) {
    factors_.reserve(factors.size());
    factors_.assign(factors.begin(), factors.end());
    updateKeys();
  }

  /** Constructor from a standard vector of factors (moves elements) */
  explicit BatchFactor(std::vector<FactorType>&& factors) {
    factors_.reserve(factors.size());
    for (auto&& f : factors) {
      factors_.push_back(std::move(f));
    }
    updateKeys();
  }

  /**
   * @brief Map-based Constructor (Varying Key2).
   * Constructs factors from a map of measurements, where the map key is the
   * second factor key.
   *
   * @param key1 The fixed first key (e.g., camera pose).
   * @param measurements Map from Key (2nd key) to Measurement.
   * @param model Noise model.
   * @param args Extra arguments passed to the factor constructor.
   */
  template <typename Measurement, typename... Args>
  BatchFactor(Key key1, const std::map<Key, Measurement>& measurements,
              const SharedNoiseModel& model, Args&&... args) {
    factors_.reserve(measurements.size());
    for (const auto& [key2, z] : measurements) {
      factors_.push_back(detail::createFactor<FactorType>(
          key1, key2, z, model, std::forward<Args>(args)...));
    }
    updateKeys();
  }

  /**
   * @brief Map-based Constructor (Varying Key1).
   * Constructs factors from a map of measurements, where the map key is the
   * first factor key.
   *
   * @param measurements Map from Key (1st key) to Measurement.
   * @param key2 The fixed second key (e.g., landmark).
   * @param model Noise model.
   * @param args Extra arguments passed to the factor constructor.
   */
  template <typename Measurement, typename... Args>
  BatchFactor(const std::map<Key, Measurement>& measurements, Key key2,
              const SharedNoiseModel& model, Args&&... args) {
    factors_.reserve(measurements.size());
    for (const auto& [key1, z] : measurements) {
      factors_.push_back(detail::createFactor<FactorType>(
          key1, key2, z, model, std::forward<Args>(args)...));
    }
    updateKeys();
  }

  /// @}
  /// @name Testable
  /// @{

  /// Print the BatchFactor
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << "BatchFactor with " << factors_.size()
              << " factors:" << std::endl;
    for (const auto& f : factors_) {
      f.print("", keyFormatter);
    }
  }

  /// Check equality with another factor.
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override {
    const This* p = dynamic_cast<const This*>(&f);
    if (!p || factors_.size() != p->factors_.size()) return false;
    for (size_t i = 0; i < factors_.size(); ++i) {
      if (!factors_[i].equals(p->factors_[i], tol)) return false;
    }
    return true;
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /**
   * Calculate the error of the factor.
   * This is the sum of the errors of all internal factors.
   */
  double error(const Values& c) const override {
    double total_error = 0.0;
    for (const auto& f : factors_) {
      total_error += f.error(c);
    }
    return total_error;
  }

  /// Get the dimension of the factor (number of rows on linearization)
  size_t dim() const override { return factors_.size() * ErrorDim; }

  void setUseHessianFactor(bool flag) { useHessianFactor_ = flag; }

  /**
   * Linearize to a single JacobianFactor.
   *
   * Optimization:
   * - Pre-calculates the total size required for the JacobianFactor.
   * - Collects all unique Keys involved across all sub-factors.
   * - Iterates linearly over factors_ (cache-friendly) to compute Jacobians.
   * - Fills the pre-allocated JacobianFactor directly.
   */
  std::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    if (factors_.empty()) return std::make_shared<JacobianFactor>();

    // 3. Allocate JacobianFactor
    // We create a VerticalBlockMatrix with the correct dimensions.
    // The total number of rows is the sum of the error dimensions of all
    // factors.
    size_t total_rows = factors_.size() * ErrorDim;
    std::vector<size_t> dims;
    dims.reserve(keyInfo_.size());
    for (const auto& info : keyInfo_) dims.push_back(info.dim);
    VerticalBlockMatrix Ab(dims, total_rows, true);
    Ab.matrix().setZero();  // Important: Initialize to zero.

    // 4. Fill the JacobianFactor
    // We reuse a vector of matrices for the Jacobians to avoid repeated
    // allocations.
    std::vector<Matrix> H(FactorType::N);
    std::vector<const void*> cache(keys().size(), nullptr);

    for (size_t i = 0; i < factors_.size(); ++i) {
      const auto& factor = factors_[i];
      size_t row_start = i * ErrorDim;

      // Legacy path (mallocs)
      // We use the factor's unwhitenedError method which fills H.
      Vector raw_error = factor.unwhitenedError(values, H);

      // Apply noise model (whitening)
      // This modifies H and raw_error in place.
      if (factor.noiseModel()) {
        factor.noiseModel()->WhitenSystem(H, raw_error);
      }

      // Place Jacobians into the large matrix
      const auto& indices_i = indices_[i];
      for (size_t k = 0; k < FactorType::N; ++k) {
        const DenseIndex index = indices_i[k];
        Ab(index).block(row_start, 0, ErrorDim, H[k].cols()) = H[k];
      }

      // Place the negative error into the RHS (last column)
      // JacobianFactor stores Ax - b, so b = -error.
      // Ab(size) gives the last block which is the RHS vector b.
      // We use block() to access the segment as a matrix block.
      Ab(keys().size()).block(row_start, 0, ErrorDim, 1) = -raw_error;
    }

    // 5. Create and return the JacobianFactor
    // We pass a Unit noise model because we have already whitened the system.
    auto jacobian = std::make_shared<JacobianFactor>(
        keys(), std::move(Ab), noiseModel::Unit::Create(total_rows));
    if (useHessianFactor_) {
      return std::make_shared<HessianFactor>(*jacobian);
    }
    return jacobian;
  }

  /// Helper to collect keys and dimensions using fold expression
  template <size_t... Is>
  void collectKeys(const FactorType& f, std::index_sequence<Is...>) {
    (keyInfo_.push_back(KeyInfo{
         f.keys()[Is],
         traits<typename FactorType::template ValueType<Is + 1>>::dimension, Is,
         0}),
     ...);
  }

  /// Update keys_ by collecting unique keys from all factors
  void updateKeys() {
    // 1. Collect all keys and their dimensions
    keyInfo_.clear();
    keyInfo_.reserve(factors_.size() * FactorType::N);

    for (const auto& f : factors_) {
      collectKeys(f, std::make_index_sequence<FactorType::N>{});
    }

    // 2. Sort and remove duplicates to get unique keys
    std::sort(keyInfo_.begin(), keyInfo_.end(),
              [](const KeyInfo& a, const KeyInfo& b) { return a.key < b.key; });
    auto isDuplicate = [](const KeyInfo& a, const KeyInfo& b) {
      if (a.key != b.key) return false;
      assert(a.dim == b.dim);
      assert(a.slot == b.slot);
      return true;
    };
    auto last = std::unique(keyInfo_.begin(), keyInfo_.end(), isDuplicate);
    keyInfo_.erase(last, keyInfo_.end());

    // 3. Fill keys_ and key information
    keys_.clear();
    keys_.reserve(keyInfo_.size());
    DenseIndex offset = 0;
    for (auto& info : keyInfo_) {
      info.offset = offset;
      keys_.push_back(info.key);
      offset += static_cast<DenseIndex>(info.dim);
    }

    // 4. Cache factor indices
    // Since keys_ is sorted, we can use binary search
    indices_.clear();
    indices_.reserve(factors_.size());
    for (const auto& f : factors_) {
      std::array<DenseIndex, FactorType::N> indices_i;
      for (size_t k = 0; k < FactorType::N; ++k) {
        auto it = std::lower_bound(keys_.begin(), keys_.end(), f.keys()[k]);
        indices_i[k] = std::distance(keys_.begin(), it);
      }
      indices_.push_back(indices_i);
    }
  }
};

}  // namespace gtsam
