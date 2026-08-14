/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmCholmodBenchmark.cpp
 * @brief Optional CHOLMOD backend for compact point-batch BAL Schur systems.
 */

#include "SfmCholmodBenchmark.h"

#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "SfmPointBatchSchur.h"
#include "TimingUtils.h"

#ifdef GTSAM_TIMING_HAS_CHOLMOD
#include <cholmod.h>
#endif

using namespace gtsam;
using symbol_shorthand::C;

namespace gtsam::timing::bal {

bool cholmodBackendAvailable() {
#ifdef GTSAM_TIMING_HAS_CHOLMOD
  return true;
#else
  return false;
#endif
}

#ifdef GTSAM_TIMING_HAS_CHOLMOD
namespace {

class CholmodContext {
  cholmod_common common_;

 public:
  CholmodContext() {
    cholmod_start(&common_);
    common_.print = 0;
  }
  ~CholmodContext() { cholmod_finish(&common_); }

  CholmodContext(const CholmodContext&) = delete;
  CholmodContext& operator=(const CholmodContext&) = delete;
  cholmod_common* get() { return &common_; }
};

class CholmodTriplet {
  cholmod_triplet* value_ = nullptr;
  cholmod_common* common_ = nullptr;

 public:
  CholmodTriplet(size_t dimension, size_t entries, cholmod_common* common)
      : value_(cholmod_allocate_triplet(dimension, dimension, entries, -1,
                                        CHOLMOD_REAL, common)),
        common_(common) {
    if (!value_) throw std::bad_alloc();
  }
  ~CholmodTriplet() {
    if (value_) cholmod_free_triplet(&value_, common_);
  }
  CholmodTriplet(const CholmodTriplet&) = delete;
  CholmodTriplet& operator=(const CholmodTriplet&) = delete;
  cholmod_triplet* get() { return value_; }
};

class CholmodSparse {
  cholmod_sparse* value_ = nullptr;
  cholmod_common* common_ = nullptr;

 public:
  CholmodSparse(cholmod_sparse* value, cholmod_common* common)
      : value_(value), common_(common) {
    if (!value_) throw std::runtime_error("CHOLMOD sparse assembly failed");
  }
  ~CholmodSparse() {
    if (value_) cholmod_free_sparse(&value_, common_);
  }
  CholmodSparse(const CholmodSparse&) = delete;
  CholmodSparse& operator=(const CholmodSparse&) = delete;
  cholmod_sparse* get() { return value_; }
};

class CholmodDense {
  cholmod_dense* value_ = nullptr;
  cholmod_common* common_ = nullptr;

 public:
  CholmodDense(size_t dimension, cholmod_common* common)
      : value_(cholmod_allocate_dense(dimension, 1, dimension, CHOLMOD_REAL,
                                      common)),
        common_(common) {
    if (!value_) throw std::bad_alloc();
  }
  CholmodDense(cholmod_dense* value, cholmod_common* common)
      : value_(value), common_(common) {
    if (!value_) throw std::runtime_error("CHOLMOD camera solve failed");
  }
  ~CholmodDense() {
    if (value_) cholmod_free_dense(&value_, common_);
  }
  CholmodDense(const CholmodDense&) = delete;
  CholmodDense& operator=(const CholmodDense&) = delete;
  cholmod_dense* get() { return value_; }
};

class CholmodFactor {
  cholmod_factor* value_ = nullptr;
  cholmod_common* common_ = nullptr;

 public:
  explicit CholmodFactor(cholmod_common* common) : common_(common) {}
  ~CholmodFactor() { reset(); }
  CholmodFactor(const CholmodFactor&) = delete;
  CholmodFactor& operator=(const CholmodFactor&) = delete;

  cholmod_factor* get() { return value_; }
  void analyze(cholmod_sparse* sparse) {
    reset();
    value_ = cholmod_analyze(sparse, common_);
    if (!value_) throw std::runtime_error("CHOLMOD symbolic analysis failed");
  }
  void reset() {
    if (value_) cholmod_free_factor(&value_, common_);
  }
};

class CholmodCameraSolver {
  CholmodContext context_;
  CholmodFactor factor_{context_.get()};
  std::vector<uint8_t> pattern_;
  size_t dimension_ = 0;

 public:
  Vector solve(const CompactCameraSystem& system) {
    const size_t dimension = 9 * system.cameraCount;
    size_t entryCount = 0;
    for (size_t row = 0; row < system.cameraCount; ++row) {
      for (size_t column = row; column < system.cameraCount; ++column) {
        const size_t block =
            upperCameraBlockIndex(row, column, system.cameraCount);
        if (!system.usedBlocks[block]) continue;
        entryCount += row == column ? 45 : 81;
      }
    }

    CholmodTriplet triplet(dimension, entryCount, context_.get());
    auto* rows = static_cast<int*>(triplet.get()->i);
    auto* columns = static_cast<int*>(triplet.get()->j);
    auto* values = static_cast<double*>(triplet.get()->x);
    size_t entry = 0;
    for (size_t blockRow = 0; blockRow < system.cameraCount; ++blockRow) {
      for (size_t blockColumn = blockRow;
           blockColumn < system.cameraCount; ++blockColumn) {
        const size_t block = upperCameraBlockIndex(
            blockRow, blockColumn, system.cameraCount);
        if (!system.usedBlocks[block]) continue;
        const Matrix99& matrix = system.blocks[block];
        if (blockRow == blockColumn) {
          for (size_t column = 0; column < 9; ++column) {
            for (size_t row = column; row < 9; ++row) {
              rows[entry] = static_cast<int>(9 * blockRow + row);
              columns[entry] = static_cast<int>(9 * blockColumn + column);
              values[entry++] = matrix(row, column);
            }
          }
        } else {
          for (size_t column = 0; column < 9; ++column) {
            for (size_t row = 0; row < 9; ++row) {
              rows[entry] = static_cast<int>(9 * blockColumn + row);
              columns[entry] = static_cast<int>(9 * blockRow + column);
              values[entry++] = matrix(column, row);
            }
          }
        }
      }
    }
    triplet.get()->nnz = entry;
    CholmodSparse sparse(
        cholmod_triplet_to_sparse(triplet.get(), entry, context_.get()),
        context_.get());

    if (!factor_.get() || dimension_ != dimension ||
        pattern_ != system.usedBlocks) {
      factor_.analyze(sparse.get());
      dimension_ = dimension;
      pattern_ = system.usedBlocks;
    }
    if (!cholmod_factorize(sparse.get(), factor_.get(), context_.get())) {
      throw std::runtime_error("CHOLMOD camera factorization failed");
    }

    CholmodDense rhs(dimension, context_.get());
    std::copy(system.rhs.data(), system.rhs.data() + dimension,
              static_cast<double*>(rhs.get()->x));
    CholmodDense solution(
        cholmod_solve(CHOLMOD_A, factor_.get(), rhs.get(), context_.get()),
        context_.get());
    return Eigen::Map<Vector>(static_cast<double*>(solution.get()->x),
                              static_cast<DenseIndex>(dimension));
  }
};

struct LinearTiming {
  size_t solves = 0;
  double assemblySeconds = 0.0;
  double solveSeconds = 0.0;
  double backSubstituteSeconds = 0.0;
};

class PointBatchSchurCholmodLevenbergMarquardtOptimizer final
    : public PointBatchSchurLevenbergMarquardtOptimizer {
  mutable CholmodCameraSolver solver_;
  mutable LinearTiming timing_;

 public:
  using PointBatchSchurLevenbergMarquardtOptimizer::
      PointBatchSchurLevenbergMarquardtOptimizer;

  VectorValues solve(const GaussianFactorGraph& graph,
                     const NonlinearOptimizerParams&) const override {
    CompactCameraSystem reduced;
    timing_.assemblySeconds += measureSeconds(
        [&] { reduced = buildPointBatchCameraSystemParallel(graph); });

    Vector cameraVector;
    timing_.solveSeconds +=
        measureSeconds([&] { cameraVector = solver_.solve(reduced); });
    ++timing_.solves;

    VectorValues cameraSolution;
    for (size_t camera = 0; camera < reduced.cameraCount; ++camera) {
      cameraSolution.insert(C(camera), cameraVector.segment<9>(9 * camera));
    }
    VectorValues solution;
    timing_.backSubstituteSeconds += measureSeconds([&] {
      solution = backSubstitutePointBatchLandmarksParallel(reduced,
                                                           cameraSolution);
    });
    return solution;
  }

  const LinearTiming& timing() const { return timing_; }
};

}  // namespace
#endif

SparseSchurOptimizationResult runPointBatchSchurCholmodOptimization(
    const NonlinearFactorGraph& graph, const Values& initial,
    const LevenbergMarquardtParams& parameters) {
#ifdef GTSAM_TIMING_HAS_CHOLMOD
  SparseSchurOptimizationResult result;
  result.initialError = graph.error(initial);
  PointBatchSchurCholmodLevenbergMarquardtOptimizer optimizer(
      graph, initial, parameters);
  result.elapsedSeconds = measureSeconds([&] { optimizer.optimize(); });
  result.finalError = optimizer.error();
  result.lmIterations = optimizer.iterations();
  result.lmInnerIterations =
      static_cast<size_t>(optimizer.getInnerIterations());
  result.linearSolves = optimizer.timing().solves;
  result.assemblySeconds = optimizer.timing().assemblySeconds;
  result.factorAndSolveSeconds = optimizer.timing().solveSeconds;
  result.backSubstituteSeconds = optimizer.timing().backSubstituteSeconds;
  return result;
#else
  (void)graph;
  (void)initial;
  (void)parameters;
  throw std::runtime_error(
      "--point-batch-schur-cholmod-only requires a build with CHOLMOD");
#endif
}

}  // namespace gtsam::timing::bal
