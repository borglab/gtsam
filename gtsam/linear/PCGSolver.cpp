/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PCGSolver.cpp
 * @brief Preconditioned Conjugate Gradient Solver for linear systems
 * @date Feb 14, 2012
 * @author Yong-Dian Jian
 * @author Sungtae An
 * @author Fan Jiang
 */

#include <gtsam/base/TaskScheduler.h>
#include <gtsam/linear/FlatGaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/VectorValues.h>

#include <Eigen/Sparse>
#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

using namespace std;

namespace gtsam {

/*****************************************************************************/
void PCGSolverParameters::print(ostream& os) const {
  Base::print(os);
  os << "PCGSolverParameters:" << endl
     << "parallel: " << parallel << endl
     << "numThreads: " << numThreads << endl;
  preconditioner->print(os);
}

/*****************************************************************************/
PCGSolver::PCGSolver(const PCGSolverParameters& p) {
  parameters_ = p;
  preconditioner_ = createPreconditioner(p.preconditioner);
}

void PCGSolverParameters::print(const std::string& s) const {
  std::cout << s << std::endl;
  std::ostringstream os;
  print(os);
  std::cout << os.str() << std::endl;
}

/**
 * @brief Compiled flat-vector operator used by GaussianFactorGraphSystem.
 *
 * Construction performs the graph traversal and keyed-to-flat mapping once so
 * repeated PCG products operate on contiguous vectors. It also computes and
 * caches the right-hand side and retains shared ownership of factors needed by
 * the compiled plans. Factors are dispatched as follows:
 *
 * - Jacobian factors are whitened once and packed into a compressed sparse
 *   Jacobian. Multiplication evaluates `J.transpose() * (J * x)` without
 *   assembling the normal-equation Hessian. Large Jacobians also retain a
 *   row-compressed view so both sparse products can be statically partitioned
 *   across a TaskScheduler.
 * - Hessian factors use precomputed scalar offsets and direct symmetric block
 *   products.
 * - Factors implementing FlatGaussianFactor, including compact batch and
 *   implicit Schur factors, use their preindexed matrix-free kernels.
 * - Other GaussianFactor implementations use the compatibility fallback.
 *
 * The block diagonal used by BlockJacobiPreconditioner is assembled lazily and
 * cached in KeyInfo ordering.
 *
 * @note This implementation reuses mutable multiplication workspace, owns a
 * scheduler, and lazily caches the block diagonal. Concurrent calls on the same
 * instance therefore require external synchronization.
 */
class GaussianFactorGraphSystem::Impl {
  struct OuterRange {
    size_t begin;
    size_t end;
  };

  /** Precomputed storage and key mappings for one Jacobian factor. */
  struct JacobianPlan {
    std::shared_ptr<const JacobianFactor> factor;
    std::vector<size_t> scalarOffsets;
    std::vector<size_t> blockSlots;
  };

  /** Precomputed storage and key mappings for one Hessian factor. */
  struct HessianPlan {
    std::shared_ptr<const HessianFactor> factor;
    std::vector<size_t> scalarOffsets;
    std::vector<size_t> blockSlots;
  };

  /** Precomputed storage and key mappings for one preindexed factor. */
  struct FlatFactorPlan {
    std::shared_ptr<const GaussianFactor> factor;
    const FlatGaussianFactor* kernels;
    std::vector<size_t> scalarOffsets;
    std::vector<size_t> blockSlots;
  };

  KeyInfo keyInfo_;  ///< Flat ordering, dimensions, and offsets for all keys.
  Vector rhs_;       ///< Cached normal-equation right-hand side, `-g(0)`.
  Eigen::SparseMatrix<double, Eigen::ColMajor, int>
      jacobian_;  ///< Compiled whitened Jacobian.
  Eigen::SparseMatrix<double, Eigen::RowMajor, int>
      rowJacobian_;                 ///< Row view used by parallel `J * x`.
  mutable Vector jacobianProduct_;  ///< Reusable workspace for `J * x`.
  std::vector<JacobianPlan> jacobianPlans_;  ///< Plans retained for diagonals.
  std::vector<HessianPlan> hessianPlans_;    ///< Direct Hessian-factor plans.
  std::vector<FlatFactorPlan> flatFactorPlans_;  ///< Preindexed factor plans.
  std::vector<OuterRange>
      flatFactorRanges_;  ///< Independent preindexed factor ranges.
  mutable std::vector<Vector>
      flatFactorProducts_;  ///< One output accumulator per factor range.
  GaussianFactorGraph fallbackFactors_;      ///< Unsupported factor types.
  mutable std::optional<std::vector<Matrix>>
      blockDiagonal_;      ///< Lazily assembled diagonal in KeyInfo ordering.
  size_t numThreads_ = 1;  ///< Effective TaskScheduler worker count.
  std::unique_ptr<TaskScheduler<void>> scheduler_;  ///< Reused worker pool.
  std::vector<OuterRange> jacobianRowRanges_;  ///< Parallel `J * x` ranges.
  std::vector<OuterRange>
      jacobianColumnRanges_;             ///< Parallel `J^T * x` ranges.
  std::vector<OuterRange> blockRanges_;  ///< Independent variable blocks.

  static constexpr size_t kMinimumParallelNonzeros = 65536;
  static constexpr size_t kMinimumParallelBlocks = 256;
  static constexpr size_t kMaximumAutomaticThreads = 8;

  /** Resolve a requested worker count, with zero selecting an automatic cap. */
  static size_t resolveThreadCount(size_t requested) {
    if (requested != 0) return requested;
    const size_t hardware = std::thread::hardware_concurrency();
    return hardware == 0 ? 1 : std::min(hardware, kMaximumAutomaticThreads);
  }

  /** Partition an outer sparse dimension into contiguous ranges. */
  static std::vector<OuterRange> makeOuterRanges(size_t count,
                                                 size_t requestedTasks) {
    const size_t taskCount = std::min(count, requestedTasks);
    std::vector<OuterRange> ranges;
    ranges.reserve(taskCount);
    for (size_t task = 0; task < taskCount; ++task) {
      const size_t begin = count * task / taskCount;
      const size_t end = count * (task + 1) / taskCount;
      if (begin != end) ranges.push_back({begin, end});
    }
    return ranges;
  }

  /** Run independent ranges on the reusable scheduler and wait for completion.
   */
  template <class FUNCTION>
  void runRanges(const std::vector<OuterRange>& ranges,
                 const FUNCTION& function) const {
    for (const OuterRange range : ranges) {
      scheduler_->enqueue(
          [range, &function] { function(range.begin, range.end); });
    }
    scheduler_->waitForAllTasks();
  }

  /** Enable parallel sparse products when the compiled Jacobian is large. */
  void configureParallelism(bool parallel, size_t requestedThreads) {
    const size_t resolvedThreads = resolveThreadCount(requestedThreads);
    const bool parallelJacobian =
        static_cast<size_t>(jacobian_.nonZeros()) >= kMinimumParallelNonzeros;
    const bool parallelBlocks = keyInfo_.size() >= kMinimumParallelBlocks;
    const bool parallelFlatFactors =
        flatFactorPlans_.size() >= kMinimumParallelBlocks;
    if (!parallel || resolvedThreads <= 1 ||
        (!parallelJacobian && !parallelBlocks && !parallelFlatFactors)) {
      return;
    }

    const size_t maximumTasks =
        std::max(keyInfo_.size(), flatFactorPlans_.size());
    numThreads_ = std::min(resolvedThreads, maximumTasks);
    if (numThreads_ <= 1) {
      numThreads_ = 1;
      return;
    }

    if (parallelJacobian) {
      rowJacobian_ = jacobian_;
      rowJacobian_.makeCompressed();
      jacobianRowRanges_ = makeOuterRanges(
          static_cast<size_t>(rowJacobian_.outerSize()), numThreads_);
      jacobianColumnRanges_ = makeOuterRanges(
          static_cast<size_t>(jacobian_.outerSize()), numThreads_);
    }
    blockRanges_ = makeOuterRanges(keyInfo_.size(), numThreads_);
    if (parallelFlatFactors) {
      flatFactorRanges_ =
          makeOuterRanges(flatFactorPlans_.size(), numThreads_);
      flatFactorProducts_.resize(flatFactorRanges_.size());
      for (Vector& product : flatFactorProducts_) {
        product = Vector::Zero(static_cast<DenseIndex>(keyInfo_.numCols()));
      }
    }
    scheduler_ = std::make_unique<TaskScheduler<void>>(numThreads_);
  }

  /** Apply matrix-free factors in parallel and reduce thread-local outputs. */
  void addFlatFactorProducts(const Vector& x, Vector* y) const {
    if (flatFactorRanges_.empty()) {
      for (const FlatFactorPlan& plan : flatFactorPlans_) {
        plan.kernels->multiplyHessianAdd(1.0, plan.scalarOffsets, x.data(),
                                         y->data());
      }
      return;
    }

    for (size_t task = 0; task < flatFactorRanges_.size(); ++task) {
      scheduler_->enqueue([&, task] {
        Vector& product = flatFactorProducts_[task];
        product.setZero();
        const OuterRange range = flatFactorRanges_[task];
        for (size_t index = range.begin; index < range.end; ++index) {
          const FlatFactorPlan& plan = flatFactorPlans_[index];
          plan.kernels->multiplyHessianAdd(1.0, plan.scalarOffsets, x.data(),
                                           product.data());
        }
      });
    }
    scheduler_->waitForAllTasks();
    for (const Vector& product : flatFactorProducts_) {
      *y += product;
    }
  }

  /** Evaluate `J * x` and `J.transpose() * (J * x)` in parallel. */
  void multiplyJacobianParallel(const Vector& x, Vector* y) const {
    const int* rowStarts = rowJacobian_.outerIndexPtr();
    const int* rowColumns = rowJacobian_.innerIndexPtr();
    const double* rowValues = rowJacobian_.valuePtr();
    const double* input = x.data();
    double* intermediate = jacobianProduct_.data();
    runRanges(jacobianRowRanges_, [&](size_t begin, size_t end) {
      for (size_t row = begin; row < end; ++row) {
        double value = 0.0;
        for (int entry = rowStarts[row]; entry < rowStarts[row + 1]; ++entry) {
          value += rowValues[entry] * input[rowColumns[entry]];
        }
        intermediate[row] = value;
      }
    });

    const int* columnStarts = jacobian_.outerIndexPtr();
    const int* columnRows = jacobian_.innerIndexPtr();
    const double* columnValues = jacobian_.valuePtr();
    double* output = y->data();
    runRanges(jacobianColumnRanges_, [&](size_t begin, size_t end) {
      for (size_t column = begin; column < end; ++column) {
        double value = 0.0;
        for (int entry = columnStarts[column]; entry < columnStarts[column + 1];
             ++entry) {
          value += columnValues[entry] * intermediate[columnRows[entry]];
        }
        output[column] = value;
      }
    });
  }

  /** Map factor-local key positions to flat scalar and ordered block slots. */
  template <class FACTOR>
  void mapFactorKeys(const FACTOR& factor, std::vector<size_t>* scalarOffsets,
                     std::vector<size_t>* blockSlots) const {
    scalarOffsets->reserve(factor.size());
    blockSlots->reserve(factor.size());
    for (size_t position = 0; position < factor.size(); ++position) {
      const Key key = factor.keys()[position];
      const auto entry = keyInfo_.find(key);
      if (entry == keyInfo_.end()) {
        throw std::invalid_argument(
            "GaussianFactorGraphSystem: factor key is absent from KeyInfo");
      }
      const size_t factorDimension = static_cast<size_t>(
          factor.getDim(factor.begin() + static_cast<DenseIndex>(position)));
      if (factorDimension != entry->second.dim) {
        throw std::invalid_argument(
            "GaussianFactorGraphSystem: factor dimension differs from "
            "KeyInfo");
      }
      scalarOffsets->push_back(entry->second.start);
      blockSlots->push_back(entry->second.index);
    }
  }

  /** Add one whitened Jacobian factor's zero-point gradient. */
  void addJacobianGradient(const JacobianPlan& plan, Vector* gradient) const {
    const auto& factor = *plan.factor;
    const auto factorRhs = factor.getb();
    for (size_t position = 0; position < factor.size(); ++position) {
      const size_t dimension = static_cast<size_t>(
          factor.getDim(factor.begin() + static_cast<DenseIndex>(position)));
      gradient
          ->segment(static_cast<DenseIndex>(plan.scalarOffsets[position]),
                    static_cast<DenseIndex>(dimension))
          .noalias() -=
          factor.getA(factor.begin() + static_cast<DenseIndex>(position))
              .transpose() *
          factorRhs;
    }
  }

  /** Add zero-point gradients for factors using the compatibility path. */
  void addFallbackGradient(Vector* gradient) const {
    if (fallbackFactors_.empty()) return;
    const VectorValues fallbackGradient = fallbackFactors_.gradientAtZero();
    for (const auto& [key, value] : fallbackGradient) {
      const auto entry = keyInfo_.find(key);
      if (entry == keyInfo_.end() ||
          static_cast<size_t>(value.size()) != entry->second.dim) {
        throw std::invalid_argument(
            "GaussianFactorGraphSystem: fallback gradient is inconsistent "
            "with KeyInfo");
      }
      gradient->segment(static_cast<DenseIndex>(entry->second.start),
                        static_cast<DenseIndex>(entry->second.dim)) += value;
    }
  }

  /** Pack all ordinary Jacobian plans into the compressed sparse Jacobian. */
  void buildSparseJacobian() {
    // Size the sparse storage and reject dimensions that exceed Eigen indices.
    size_t rowCount = 0;
    size_t scalarEntryCount = 0;
    for (const JacobianPlan& plan : jacobianPlans_) {
      rowCount += plan.factor->rows();
      for (size_t position = 0; position < plan.factor->size(); ++position) {
        scalarEntryCount +=
            plan.factor->rows() *
            static_cast<size_t>(plan.factor->getDim(
                plan.factor->begin() + static_cast<DenseIndex>(position)));
      }
    }
    if (rowCount > static_cast<size_t>(std::numeric_limits<int>::max()) ||
        keyInfo_.numCols() >
            static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw std::length_error(
          "GaussianFactorGraphSystem: sparse Jacobian exceeds Eigen index "
          "range");
    }

    std::vector<Eigen::Triplet<double, int>> entries;
    entries.reserve(scalarEntryCount);

    // Copy each whitened dense block into the global flat Jacobian.
    size_t rowOffset = 0;
    for (const JacobianPlan& plan : jacobianPlans_) {
      const auto& factor = *plan.factor;
      for (size_t position = 0; position < factor.size(); ++position) {
        const auto block =
            factor.getA(factor.begin() + static_cast<DenseIndex>(position));
        const size_t columnOffset = plan.scalarOffsets[position];
        for (DenseIndex column = 0; column < block.cols(); ++column) {
          for (DenseIndex row = 0; row < block.rows(); ++row) {
            const double value = block(row, column);
            if (value != 0.0) {
              entries.emplace_back(
                  static_cast<int>(rowOffset + static_cast<size_t>(row)),
                  static_cast<int>(columnOffset + static_cast<size_t>(column)),
                  value);
            }
          }
        }
      }
      rowOffset += factor.rows();
    }

    jacobian_.resize(static_cast<int>(rowCount),
                     static_cast<int>(keyInfo_.numCols()));
    jacobian_.setFromTriplets(entries.begin(), entries.end());
    jacobian_.makeCompressed();
    jacobianProduct_.resize(static_cast<DenseIndex>(rowCount));
  }

  /** Compile one factor into its specialized plan or the fallback graph. */
  void compileFactor(const GaussianFactor::shared_ptr& factor,
                     Vector* gradient) {
    if (const auto* kernels =
            dynamic_cast<const FlatGaussianFactor*>(factor.get())) {
      FlatFactorPlan plan;
      plan.factor = factor;
      plan.kernels = kernels;
      mapFactorKeys(*plan.factor, &plan.scalarOffsets, &plan.blockSlots);
      plan.kernels->gradientAtZeroAdd(plan.scalarOffsets, gradient->data());
      flatFactorPlans_.push_back(std::move(plan));
      return;
    }

    if (auto hessian = std::dynamic_pointer_cast<HessianFactor>(factor)) {
      HessianPlan plan;
      plan.factor = std::move(hessian);
      mapFactorKeys(*plan.factor, &plan.scalarOffsets, &plan.blockSlots);
      for (size_t position = 0; position < plan.factor->size(); ++position) {
        const size_t dimension = static_cast<size_t>(plan.factor->getDim(
            plan.factor->begin() + static_cast<DenseIndex>(position)));
        gradient
            ->segment(static_cast<DenseIndex>(plan.scalarOffsets[position]),
                      static_cast<DenseIndex>(dimension))
            .noalias() -= plan.factor->linearTerm(
            plan.factor->begin() + static_cast<DenseIndex>(position));
      }
      hessianPlans_.push_back(std::move(plan));
      return;
    }

    if (auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(factor)) {
      JacobianPlan plan;
      plan.factor = jacobian->get_model() && !jacobian->get_model()->isUnit()
                        ? std::make_shared<JacobianFactor>(jacobian->whiten())
                        : std::move(jacobian);
      mapFactorKeys(*plan.factor, &plan.scalarOffsets, &plan.blockSlots);
      addJacobianGradient(plan, gradient);
      jacobianPlans_.push_back(std::move(plan));
      return;
    }

    fallbackFactors_.push_back(factor);
  }

  /** Add products for explicit symmetric Hessian factor plans. */
  void addHessianProducts(const Vector& x, Vector* y) const {
    for (const HessianPlan& plan : hessianPlans_) {
      const auto& factor = *plan.factor;
      for (DenseIndex i = 0; i < static_cast<DenseIndex>(factor.size()); ++i) {
        const DenseIndex dimensionI = factor.getDim(factor.begin() + i);
        const DenseIndex offsetI =
            static_cast<DenseIndex>(plan.scalarOffsets[i]);
        const auto xI = x.segment(offsetI, dimensionI);
        y->segment(offsetI, dimensionI).noalias() +=
            factor.info().diagonalBlock(i) * xI;
        for (DenseIndex j = i + 1; j < static_cast<DenseIndex>(factor.size());
             ++j) {
          const DenseIndex dimensionJ = factor.getDim(factor.begin() + j);
          const DenseIndex offsetJ =
              static_cast<DenseIndex>(plan.scalarOffsets[j]);
          const auto block = factor.info().aboveDiagonalBlock(i, j);
          y->segment(offsetI, dimensionI).noalias() +=
              block * x.segment(offsetJ, dimensionJ);
          y->segment(offsetJ, dimensionJ).noalias() += block.transpose() * xI;
        }
      }
    }
  }

  /** Add products from GaussianFactor implementations on the fallback path. */
  void addFallbackProduct(const Vector& x, Vector* y) const {
    if (fallbackFactors_.empty()) return;
    const VectorValues vectorValuesX = buildVectorValues(x, keyInfo_);
    VectorValues vectorValuesY = keyInfo_.x0();
    fallbackFactors_.multiplyHessianAdd(1.0, vectorValuesX, vectorValuesY);
    for (const auto& [key, value] : vectorValuesY) {
      const auto entry = keyInfo_.find(key);
      y->segment(static_cast<DenseIndex>(entry->second.start),
                 static_cast<DenseIndex>(entry->second.dim)) += value;
    }
  }

 public:
  /**
   * Compile a factor graph for the supplied flat key layout.
   *
   * @param graph Gaussian factors to compile and retain.
   * @param keyInfo Ordering, dimensions, and flat offsets for graph keys.
   * @param parallel Whether sufficiently large kernels may run concurrently.
   * @param numThreads Requested worker count; zero selects automatically.
   */
  Impl(const GaussianFactorGraph& graph, const KeyInfo& keyInfo, bool parallel,
       size_t numThreads)
      : keyInfo_(keyInfo), rhs_(Vector::Zero(keyInfo.numCols())) {
    Vector gradient = Vector::Zero(keyInfo.numCols());

    jacobianPlans_.reserve(graph.size());
    hessianPlans_.reserve(graph.size());
    flatFactorPlans_.reserve(graph.size());

    // Dispatch supported factors into specialized plans and retain the rest.
    for (const auto& factor : graph) {
      if (!factor) continue;
      compileFactor(factor, &gradient);
    }

    // Finalize the right-hand side and sparse execution infrastructure.
    addFallbackGradient(&gradient);
    rhs_ = -gradient;
    buildSparseJacobian();
    configureParallelism(parallel, numThreads);
  }

  /**
   * Evaluate the normal-equation product `y = H * x`.
   *
   * @param x Input vector in KeyInfo ordering.
   * @param y Output vector, resized and overwritten. It may alias @p x.
   */
  void multiply(const Vector& x, Vector* y) const {
    if (x.size() != static_cast<DenseIndex>(keyInfo_.numCols())) {
      throw std::invalid_argument(
          "GaussianFactorGraphSystem::multiply: input dimension mismatch");
    }
    if (x.data() == y->data()) {
      const Vector input = x;
      multiply(input, y);
      return;
    }
    y->setZero(static_cast<DenseIndex>(keyInfo_.numCols()));

    // Apply the compiled sparse Jacobian contribution J.transpose()*J*x.
    if (!jacobianRowRanges_.empty()) {
      multiplyJacobianParallel(x, y);
    } else if (jacobian_.rows() != 0) {
      jacobianProduct_.noalias() = jacobian_ * x;
      y->noalias() += jacobian_.transpose() * jacobianProduct_;
    }

    // Apply explicit symmetric Hessian blocks without materializing H.
    addHessianProducts(x, y);

    addFlatFactorProducts(x, y);

    // Preserve compatibility for all remaining GaussianFactor subclasses.
    addFallbackProduct(x, y);
  }

  /** Return the cached normal-equation right-hand side. */
  const Vector& rhs() const { return rhs_; }

  /** Return the effective scheduler worker count. */
  size_t numThreads() const { return numThreads_; }

  /** Apply independent work to variable-block ranges when profitable. */
  template <class FUNCTION>
  bool runBlockRanges(size_t blockCount, const FUNCTION& function) const {
    if (!scheduler_ || blockCount < kMinimumParallelBlocks ||
        blockCount != keyInfo_.size()) {
      return false;
    }
    runRanges(blockRanges_, function);
    return true;
  }

  /**
   * Return the lazily assembled Hessian diagonal blocks in KeyInfo ordering.
   */
  const std::vector<Matrix>& hessianBlockDiagonal() const {
    if (blockDiagonal_) return *blockDiagonal_;

    // Allocate one zero block per key in KeyInfo ordering.
    const std::vector<size_t> dimensions = keyInfo_.colSpec();
    blockDiagonal_.emplace(dimensions.size());
    for (size_t slot = 0; slot < dimensions.size(); ++slot) {
      (*blockDiagonal_)[slot] =
          Matrix::Zero(static_cast<DenseIndex>(dimensions[slot]),
                       static_cast<DenseIndex>(dimensions[slot]));
    }

    // Accumulate diagonal contributions from each specialized factor plan.
    for (const JacobianPlan& plan : jacobianPlans_) {
      const auto& factor = *plan.factor;
      for (size_t position = 0; position < factor.size(); ++position) {
        const auto block =
            factor.getA(factor.begin() + static_cast<DenseIndex>(position));
        (*blockDiagonal_)[plan.blockSlots[position]].noalias() +=
            block.transpose() * block;
      }
    }

    for (const HessianPlan& plan : hessianPlans_) {
      for (DenseIndex position = 0;
           position < static_cast<DenseIndex>(plan.factor->size());
           ++position) {
        (*blockDiagonal_)[plan.blockSlots[position]] +=
            plan.factor->info().diagonalBlock(position);
      }
    }

    for (const FlatFactorPlan& plan : flatFactorPlans_) {
      plan.kernels->hessianBlockDiagonalAdd(plan.blockSlots, &*blockDiagonal_);
    }

    // Complete the diagonal through the general compatibility graph.
    if (!fallbackFactors_.empty()) {
      const std::map<Key, Matrix> fallbackDiagonal =
          fallbackFactors_.hessianBlockDiagonal();
      for (const auto& [key, block] : fallbackDiagonal) {
        (*blockDiagonal_)[keyInfo_.at(key).index] += block;
      }
    }
    return *blockDiagonal_;
  }
};

/*****************************************************************************/
VectorValues PCGSolver::optimize(const GaussianFactorGraph& gfg,
                                 const KeyInfo& keyInfo,
                                 const std::map<Key, Vector>& lambda,
                                 const VectorValues& initial) {
  return optimizeDetailed(gfg, keyInfo, lambda, initial, false).solution;
}

/*****************************************************************************/
PCGSolverResult PCGSolver::optimizeDetailed(const GaussianFactorGraph& gfg,
                                            bool collectResidualHistory) {
  const KeyInfo keyInfo(gfg);
  return optimizeDetailed(gfg, keyInfo, {}, keyInfo.x0(),
                          collectResidualHistory);
}

/*****************************************************************************/
PCGSolverResult PCGSolver::optimizeDetailed(const GaussianFactorGraph& gfg,
                                            const KeyInfo& keyInfo,
                                            const std::map<Key, Vector>& lambda,
                                            const VectorValues& initial,
                                            bool collectResidualHistory) {
  using Clock = std::chrono::steady_clock;

  // Compile the graph into its flat matrix-free operator.
  const auto operatorStart = Clock::now();
  GaussianFactorGraphSystem system(gfg, *preconditioner_, keyInfo, lambda,
                                   parameters_.parallel,
                                   parameters_.numThreads);
  const auto operatorEnd = Clock::now();

  // Build block Jacobi directly from compiled blocks when available.
  const auto preconditionerStart = Clock::now();
  if (auto blockJacobi = std::dynamic_pointer_cast<BlockJacobiPreconditioner>(
          preconditioner_)) {
    blockJacobi->build(system.hessianBlockDiagonal(), keyInfo);
  } else {
    preconditioner_->build(gfg, keyInfo, lambda);
  }
  const auto preconditionerEnd = Clock::now();

  // Run PCG in flat storage and restore the keyed public representation.
  const auto solveStart = Clock::now();
  const Vector x0 = initial.vector(keyInfo.ordering());
  auto cgResult = preconditionedConjugateGradientDetailed(
      system, x0, parameters_, collectResidualHistory);
  VectorValues solution = buildVectorValues(cgResult.solution, keyInfo);
  const auto solveEnd = Clock::now();

  PCGSolverResult result;
  result.solution = std::move(solution);
  result.stats = std::move(cgResult.stats);
  result.operatorSetupSeconds =
      std::chrono::duration<double>(operatorEnd - operatorStart).count();
  result.preconditionerSetupSeconds =
      std::chrono::duration<double>(preconditionerEnd - preconditionerStart)
          .count();
  result.solveSeconds =
      std::chrono::duration<double>(solveEnd - solveStart).count();
  return result;
}

/*****************************************************************************/
GaussianFactorGraphSystem::GaussianFactorGraphSystem(
    const GaussianFactorGraph& gfg, const Preconditioner& preconditioner,
    const KeyInfo& keyInfo, const std::map<Key, Vector>& lambda, bool parallel,
    size_t numThreads)
    : preconditioner_(preconditioner),
      impl_(std::make_shared<Impl>(gfg, keyInfo, parallel, numThreads)) {
  (void)lambda;
}

/*****************************************************************************/
void GaussianFactorGraphSystem::residual(const Vector& x, Vector& r) const {
  multiply(x, r);
  r = -r;
  r += impl_->rhs();
}

/*****************************************************************************/
void GaussianFactorGraphSystem::multiply(const Vector& x, Vector& AtAx) const {
  impl_->multiply(x, &AtAx);
}

/*****************************************************************************/
void GaussianFactorGraphSystem::getb(Vector& b) const { b = impl_->rhs(); }

/*****************************************************************************/
size_t GaussianFactorGraphSystem::numThreads() const {
  return impl_->numThreads();
}

/*****************************************************************************/
const std::vector<Matrix>& GaussianFactorGraphSystem::hessianBlockDiagonal()
    const {
  return impl_->hessianBlockDiagonal();
}

/**********************************************************************************/
void GaussianFactorGraphSystem::leftPrecondition(const Vector& x,
                                                 Vector& y) const {
  // For a preconditioner M = L*L^T
  // Calculate y = L^{-1} x
  if (const auto* blockJacobi =
          dynamic_cast<const BlockJacobiPreconditioner*>(&preconditioner_)) {
    y = x;
    if (impl_->runBlockRanges(
            blockJacobi->dims_.size(), [&](size_t begin, size_t end) {
              blockJacobi->solveInPlaceRange(y, begin, end, false);
            })) {
      return;
    }
    blockJacobi->solveInPlaceRange(y, 0, blockJacobi->dims_.size(), false);
    return;
  }
  preconditioner_.solve(x, y);
}

/**********************************************************************************/
void GaussianFactorGraphSystem::rightPrecondition(const Vector& x,
                                                  Vector& y) const {
  // For a preconditioner M = L*L^T
  // Calculate y = L^{-T} x
  if (const auto* blockJacobi =
          dynamic_cast<const BlockJacobiPreconditioner*>(&preconditioner_)) {
    y = x;
    if (impl_->runBlockRanges(
            blockJacobi->dims_.size(), [&](size_t begin, size_t end) {
              blockJacobi->solveInPlaceRange(y, begin, end, true);
            })) {
      return;
    }
    blockJacobi->solveInPlaceRange(y, 0, blockJacobi->dims_.size(), true);
    return;
  }
  preconditioner_.transposeSolve(x, y);
}

/**********************************************************************************/
void GaussianFactorGraphSystem::scal(const double alpha, Vector& x) const {
  x *= alpha;
}
double GaussianFactorGraphSystem::dot(const Vector& x, const Vector& y) const {
  return x.dot(y);
}
void GaussianFactorGraphSystem::axpy(const double alpha, const Vector& x,
                                     Vector& y) const {
  y += alpha * x;
}
/**********************************************************************************/
VectorValues buildVectorValues(const Vector& v, const Ordering& ordering,
                               const map<Key, size_t>& dimensions) {
  VectorValues result;

  DenseIndex offset = 0;
  for (size_t i = 0; i < ordering.size(); ++i) {
    const Key key = ordering[i];
    map<Key, size_t>::const_iterator it = dimensions.find(key);
    if (it == dimensions.end()) {
      throw invalid_argument(
          "buildVectorValues: inconsistent ordering and dimensions");
    }
    const size_t dim = it->second;
    result.emplace(key, v.segment(offset, dim));
    offset += dim;
  }

  return result;
}

}  // namespace gtsam
