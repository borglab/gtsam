#pragma once

#include <gtsam/base/FastMap.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gtsam::cuda {

struct SparseJacobianColumnBlock {
  Key key = 0;
  int dimension = 0;
  int columnBegin = 0;
};

class GTSAM_EXPORT SparseJacobianColumnLayout {
 public:
  explicit SparseJacobianColumnLayout(const Values& values);

  const SparseJacobianColumnBlock& at(Key key) const;
  const std::vector<SparseJacobianColumnBlock>& blocks() const;
  int totalColumns() const;
  bool matches(const Values& values) const;
  VectorValues toVectorValues(const Vector& flatDelta) const;

 private:
  std::vector<SparseJacobianColumnBlock> blocks_;
  FastMap<Key, size_t> keyToBlock_;
  int totalColumns_ = 0;
};

struct SparseJacobianBlockWritePlan {
  Key key = 0;
  size_t localBlockIndex = 0;
  int width = 0;
  int globalColumnBegin = 0;
  int valueOffsetWithinRow = 0;
};

struct SparseJacobianFactorWritePlan {
  int rowBegin = 0;
  int rowCount = 0;
  int nonzerosPerRow = 0;
  bool sendable = true;
  std::vector<SparseJacobianBlockWritePlan> blocks;
};

class GTSAM_EXPORT SparseJacobianPlan {
 public:
  SparseJacobianPlan(const NonlinearFactorGraph& graph,
                     const SparseJacobianColumnLayout& columns);

  int rows() const;
  int columns() const;
  int nonzeros() const;
  const std::vector<int>& rowPointers() const;
  const std::vector<int>& columnIndices() const;
  const SparseJacobianFactorWritePlan& factor(size_t index) const;
  const std::vector<SparseJacobianFactorWritePlan>& factors() const;
  uint64_t structuralFingerprint() const;
  bool matches(const NonlinearFactorGraph& graph,
               const SparseJacobianColumnLayout& columns) const;

 private:
  int rows_ = 0;
  int columns_ = 0;
  std::vector<int> rowPointers_{0};
  std::vector<int> columnIndices_;
  std::vector<SparseJacobianFactorWritePlan> factors_;
  uint64_t structuralFingerprint_ = 0;

  std::vector<SparseJacobianColumnBlock> columnBlocks_;
  std::vector<bool> factorIsNull_;
};

}  // namespace gtsam::cuda
