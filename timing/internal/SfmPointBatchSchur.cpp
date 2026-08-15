/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmPointBatchSchur.cpp
 * @brief Backend-neutral compact Schur assembly for explicit-point BAL.
 */

#include "SfmPointBatchSchur.h"

#include <gtsam/base/TaskScheduler.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::P;

namespace gtsam::timing::bal {
namespace {

size_t parallelWorkerCount(size_t taskCount, size_t requestedThreads = 0) {
  const size_t hardwareThreads = std::thread::hardware_concurrency();
  const size_t availableThreads =
      requestedThreads == 0 ? hardwareThreads : requestedThreads;
  return std::max<size_t>(
      1, std::min(taskCount, std::max<size_t>(1, availableThreads)));
}

struct CameraAccumulator {
  CameraHessianBlocks blocks;
  std::vector<uint8_t> usedBlocks;
  Vector rhs;
  std::vector<Matrix39, Eigen::aligned_allocator<Matrix39>> pointCamera;
};

CompactCameraSystem::Landmark preparePointBatch(
    const std::shared_ptr<const PointBatchJacobian>& batch,
    const Matrix3& pointInformation,
    const Vector3& pointInformationRhs) {
  if (batch->get_model() && !batch->get_model()->isUnit()) {
    throw std::runtime_error(
        "Point-batch sparse Schur requires pre-whitened compact factors");
  }
  if (batch->rowSlots().empty()) {
    throw std::runtime_error("Point-batch sparse Schur found an empty batch");
  }

  Matrix3 pointHessian = pointInformation;
  const Key landmark = batch->keys().at(batch->rowSlots().front()[1]);
  if (symbolChr(landmark) != 'p') {
    throw std::runtime_error(
        "Point-batch sparse Schur found a batch without a landmark");
  }
  for (size_t row = 0; row < batch->rowSlots().size(); ++row) {
    const auto& slots = batch->rowSlots()[row];
    if (symbolChr(batch->keys().at(slots[0])) != 'c' ||
        batch->keys().at(slots[1]) != landmark) {
      throw std::runtime_error(
          "Point-batch sparse Schur requires camera-point BAL rows");
    }
    pointHessian.noalias() +=
        batch->block<1>(row).transpose() * batch->block<1>(row);
  }
  const Eigen::LLT<Matrix3> factorization(pointHessian);
  if (factorization.info() != Eigen::Success) {
    throw std::runtime_error(
        "Point-batch sparse Schur found a singular landmark block");
  }
  return {batch, landmark, factorization.solve(Matrix3::Identity()),
          pointInformationRhs};
}

void accumulatePointSchur(const CompactCameraSystem::Landmark& landmark,
                          size_t cameraCount,
                          CameraAccumulator* accumulator) {
  const PointBatchJacobian& batch = *landmark.batch;
  const size_t measurementCount = batch.rowSlots().size();
  accumulator->pointCamera.resize(measurementCount);
  Vector3 pointRhs = landmark.pointInformationRhs;

  for (size_t measurement = 0; measurement < measurementCount;
       ++measurement) {
    const Matrix23& pointJacobian = batch.block<1>(measurement);
    accumulator->pointCamera[measurement].noalias() =
        pointJacobian.transpose() * batch.block<0>(measurement);
    pointRhs.noalias() +=
        pointJacobian.transpose() * batch.rowRhs(measurement);
  }
  for (size_t measurement = 0; measurement < measurementCount;
       ++measurement) {
    const auto& slots = batch.rowSlots()[measurement];
    const size_t camera = symbolIndex(batch.keys().at(slots[0]));
    if (camera >= cameraCount) {
      throw std::runtime_error(
          "Point-batch sparse Schur found a non-contiguous camera index");
    }
    accumulator->rhs.segment<9>(9 * camera).noalias() +=
        batch.block<0>(measurement).transpose() * batch.rowRhs(measurement) -
        accumulator->pointCamera[measurement].transpose() *
            landmark.pointCovariance * pointRhs;
  }

  for (size_t i = 0; i < measurementCount; ++i) {
    const size_t cameraI =
        symbolIndex(batch.keys().at(batch.rowSlots()[i][0]));
    for (size_t j = i; j < measurementCount; ++j) {
      const size_t cameraJ =
          symbolIndex(batch.keys().at(batch.rowSlots()[j][0]));
      Matrix99 contribution =
          -accumulator->pointCamera[i].transpose() *
          landmark.pointCovariance * accumulator->pointCamera[j];
      if (i == j) {
        contribution.noalias() +=
            batch.block<0>(i).transpose() * batch.block<0>(i);
      }
      const size_t block =
          upperCameraBlockIndex(cameraI, cameraJ, cameraCount);
      accumulator->blocks[block] +=
          cameraI <= cameraJ ? contribution : contribution.transpose();
      accumulator->usedBlocks[block] = 1;
    }
  }
}

template <int Dimension>
GaussianFactor::shared_ptr createDampingBatch(
    const VectorValues& sqrtHessianDiagonal, double lambda) {
  using DampingBatch = BatchJacobianFactor<Dimension, Dimension>;
  KeyVector keys;
  for (const auto& [key, diagonal] : sqrtHessianDiagonal) {
    if (diagonal.size() == Dimension) keys.push_back(key);
  }
  auto batch = std::make_shared<DampingBatch>(
      keys, std::vector<size_t>(keys.size(), Dimension));
  batch->reserve(keys.size());
  const double sqrtLambda = std::sqrt(lambda);
  for (size_t slot = 0; slot < keys.size(); ++slot) {
    const Eigen::Matrix<double, Dimension, Dimension> block =
        sqrtLambda * sqrtHessianDiagonal.at(keys[slot]).asDiagonal();
    batch->addUnaryRow(static_cast<DenseIndex>(slot), block,
                       Eigen::Matrix<double, Dimension, 1>::Zero());
  }
  return batch;
}

}  // namespace

size_t upperCameraBlockIndex(size_t row, size_t column,
                             size_t cameraCount) {
  if (row > column) std::swap(row, column);
  if (column >= cameraCount) {
    throw std::out_of_range("Camera block index exceeds camera count");
  }
  return row * cameraCount - row * (row - 1) / 2 + (column - row);
}

CompactCameraSystem buildPointBatchCameraSystemParallel(
    const GaussianFactorGraph& graph, size_t numThreads) {
  std::vector<std::shared_ptr<const PointBatchJacobian>> pointBatches;
  std::vector<GaussianFactor::shared_ptr> pointDamping;
  std::vector<GaussianFactor::shared_ptr> cameraDamping;
  std::vector<Matrix3, Eigen::aligned_allocator<Matrix3>> pointDampingBlocks;
  std::vector<uint8_t> pointDampingPresent;
  std::vector<Matrix99, Eigen::aligned_allocator<Matrix99>>
      cameraDampingBlocks;
  std::vector<uint8_t> cameraDampingPresent;

  const auto resizeForPoint = [&](size_t point) {
    if (pointBatches.size() <= point) {
      pointBatches.resize(point + 1);
      pointDamping.resize(point + 1);
      pointDampingBlocks.resize(point + 1, Matrix3::Zero());
      pointDampingPresent.resize(point + 1, 0);
    }
  };

  for (const auto& factor : graph) {
    if (!factor) continue;
    if (const auto damping =
            std::dynamic_pointer_cast<PointDampingBatch>(factor)) {
      for (size_t row = 0; row < damping->rowSlots().size(); ++row) {
        const Key key = damping->keys().at(damping->rowSlots()[row][0]);
        const size_t point = symbolIndex(key);
        resizeForPoint(point);
        pointDampingBlocks[point].noalias() =
            damping->block<0>(row).transpose() * damping->block<0>(row);
        pointDampingPresent[point] = 1;
      }
      continue;
    }
    if (const auto damping =
            std::dynamic_pointer_cast<CameraDampingBatch>(factor)) {
      for (size_t row = 0; row < damping->rowSlots().size(); ++row) {
        const Key key = damping->keys().at(damping->rowSlots()[row][0]);
        const size_t camera = symbolIndex(key);
        if (cameraDampingBlocks.size() <= camera) {
          cameraDampingBlocks.resize(camera + 1, Matrix99::Zero());
          cameraDampingPresent.resize(camera + 1, 0);
        }
        cameraDampingBlocks[camera].noalias() =
            damping->block<0>(row).transpose() * damping->block<0>(row);
        cameraDampingPresent[camera] = 1;
      }
      continue;
    }
    if (const auto batch =
            std::dynamic_pointer_cast<PointBatchJacobian>(factor)) {
      if (batch->rowSlots().empty()) continue;
      const Key landmark = batch->keys().at(batch->rowSlots().front()[1]);
      const size_t point = symbolIndex(landmark);
      resizeForPoint(point);
      if (pointBatches[point]) {
        throw std::runtime_error(
            "Point-batch sparse Schur requires one batch per landmark");
      }
      pointBatches[point] = std::move(batch);
      continue;
    }
    if (factor->size() != 1) {
      throw std::runtime_error(
          "Point-batch sparse Schur found an unsupported linear factor");
    }
    const Key key = factor->keys().front();
    if (symbolChr(key) == 'p') {
      const size_t point = symbolIndex(key);
      resizeForPoint(point);
      pointDamping[point] = factor;
    } else if (symbolChr(key) == 'c') {
      const size_t camera = symbolIndex(key);
      if (cameraDamping.size() <= camera) cameraDamping.resize(camera + 1);
      cameraDamping[camera] = factor;
    } else {
      throw std::runtime_error(
          "Point-batch sparse Schur found an unsupported variable type");
    }
  }

  CompactCameraSystem system;
  system.cameraCount =
      std::max(cameraDamping.size(), cameraDampingBlocks.size());
  if (system.cameraCount == 0) {
    throw std::runtime_error("Point-batch sparse Schur found no cameras");
  }
  const size_t blockCount =
      system.cameraCount * (system.cameraCount + 1) / 2;
  system.blocks.assign(blockCount, Matrix99::Zero());
  system.usedBlocks.assign(blockCount, 0);
  system.rhs = Vector::Zero(9 * system.cameraCount);

  std::vector<size_t> activePoints;
  activePoints.reserve(pointBatches.size());
  for (size_t point = 0; point < pointBatches.size(); ++point) {
    if (pointBatches[point]) {
      activePoints.push_back(point);
    } else if (pointDamping[point] || pointDampingPresent[point]) {
      system.zeroLandmarks.push_back(P(point));
    }
  }
  system.landmarks.resize(activePoints.size());

  const size_t workerCount =
      parallelWorkerCount(activePoints.size(), numThreads);
  std::vector<CameraAccumulator> accumulators(workerCount);
  for (CameraAccumulator& accumulator : accumulators) {
    accumulator.blocks.assign(blockCount, Matrix99::Zero());
    accumulator.usedBlocks.assign(blockCount, 0);
    accumulator.rhs = Vector::Zero(9 * system.cameraCount);
  }

  TaskScheduler<void> scheduler(workerCount);
  for (size_t worker = 0; worker < workerCount; ++worker) {
    scheduler.enqueue([&, worker] {
      const size_t begin = activePoints.size() * worker / workerCount;
      const size_t end = activePoints.size() * (worker + 1) / workerCount;
      for (size_t active = begin; active < end; ++active) {
        const size_t point = activePoints[active];
        Matrix3 information = Matrix3::Zero();
        Vector3 informationRhs = Vector3::Zero();
        if (pointDampingPresent[point]) {
          information = pointDampingBlocks[point];
        } else if (pointDamping[point]) {
          information = pointDamping[point]->information();
          const VectorValues gradient = pointDamping[point]->gradientAtZero();
          if (gradient.exists(P(point))) {
            informationRhs = -gradient.at(P(point));
          }
        }
        system.landmarks[active] = preparePointBatch(
            pointBatches[point], information, informationRhs);
        accumulatePointSchur(system.landmarks[active], system.cameraCount,
                             &accumulators[worker]);
      }
    });
  }
  scheduler.waitForAllTasks();

  for (const CameraAccumulator& accumulator : accumulators) {
    system.rhs += accumulator.rhs;
    for (size_t block = 0; block < blockCount; ++block) {
      if (!accumulator.usedBlocks[block]) continue;
      system.blocks[block] += accumulator.blocks[block];
      system.usedBlocks[block] = 1;
    }
  }

  for (size_t camera = 0; camera < cameraDamping.size(); ++camera) {
    if (!cameraDamping[camera]) continue;
    const Matrix information = cameraDamping[camera]->information();
    if (information.rows() != 9 || information.cols() != 9) {
      throw std::runtime_error(
          "Point-batch sparse Schur found invalid camera damping");
    }
    const size_t diagonal =
        upperCameraBlockIndex(camera, camera, system.cameraCount);
    system.blocks[diagonal] += information;
    system.usedBlocks[diagonal] = 1;
    const VectorValues gradient = cameraDamping[camera]->gradientAtZero();
    if (gradient.exists(C(camera))) {
      system.rhs.segment<9>(9 * camera) -= gradient.at(C(camera));
    }
  }
  for (size_t camera = 0; camera < cameraDampingBlocks.size(); ++camera) {
    if (!cameraDampingPresent[camera]) continue;
    const size_t diagonal =
        upperCameraBlockIndex(camera, camera, system.cameraCount);
    system.blocks[diagonal] += cameraDampingBlocks[camera];
    system.usedBlocks[diagonal] = 1;
  }
  return system;
}

VectorValues backSubstitutePointBatchLandmarksParallel(
    const CompactCameraSystem& system,
    const VectorValues& cameraSolution) {
  std::vector<Vector3, Eigen::aligned_allocator<Vector3>> pointSolutions(
      system.landmarks.size());
  const size_t workerCount = parallelWorkerCount(system.landmarks.size());
  TaskScheduler<void> scheduler(workerCount);
  for (size_t worker = 0; worker < workerCount; ++worker) {
    scheduler.enqueue([&, worker] {
      const size_t begin = system.landmarks.size() * worker / workerCount;
      const size_t end = system.landmarks.size() * (worker + 1) / workerCount;
      for (size_t index = begin; index < end; ++index) {
        const CompactCameraSystem::Landmark& landmark =
            system.landmarks[index];
        const PointBatchJacobian& batch = *landmark.batch;
        Vector3 pointRhs = landmark.pointInformationRhs;
        for (size_t row = 0; row < batch.rowSlots().size(); ++row) {
          const Key camera = batch.keys().at(batch.rowSlots()[row][0]);
          const Vector2 residual =
              batch.rowRhs(row) -
              batch.block<0>(row) * cameraSolution.at(camera);
          pointRhs.noalias() +=
              batch.block<1>(row).transpose() * residual;
        }
        pointSolutions[index].noalias() =
            landmark.pointCovariance * pointRhs;
      }
    });
  }
  scheduler.waitForAllTasks();

  VectorValues solution = cameraSolution;
  for (size_t index = 0; index < system.landmarks.size(); ++index) {
    solution.insert(system.landmarks[index].key, pointSolutions[index]);
  }
  for (const Key landmark : system.zeroLandmarks) {
    solution.insert(landmark, Vector3::Zero());
  }
  return solution;
}

GaussianFactorGraph
PointBatchSchurLevenbergMarquardtOptimizer::buildDampedSystem(
    const GaussianFactorGraph& linear,
    const VectorValues& sqrtHessianDiagonal) const {
  GaussianFactorGraph damped = linear;
  damped.reserve(linear.size() + 2);
  damped.push_back(createDampingBatch<3>(sqrtHessianDiagonal, lambda()));
  damped.push_back(createDampingBatch<9>(sqrtHessianDiagonal, lambda()));
  return damped;
}

double PointBatchSchurLevenbergMarquardtOptimizer::linearDeltaError(
    const GaussianFactorGraph& linear, const VectorValues& delta,
    double* oldError, double* newError) const {
  const size_t workerCount = parallelWorkerCount(linear.size());
  std::vector<double> oldPartials(workerCount, 0.0);
  std::vector<double> newPartials(workerCount, 0.0);
  std::vector<double> deltaPartials(workerCount, 0.0);
  TaskScheduler<void> scheduler(workerCount);
  for (size_t worker = 0; worker < workerCount; ++worker) {
    scheduler.enqueue([&, worker] {
      const size_t begin = linear.size() * worker / workerCount;
      const size_t end = linear.size() * (worker + 1) / workerCount;
      for (size_t index = begin; index < end; ++index) {
        const auto& factor = linear.at(index);
        if (!factor) continue;
        double factorOld = 0.0, factorNew = 0.0;
        deltaPartials[worker] +=
            factor->deltaError(delta, &factorOld, &factorNew);
        oldPartials[worker] += factorOld;
        newPartials[worker] += factorNew;
      }
    });
  }
  scheduler.waitForAllTasks();
  double oldTotal = 0.0, newTotal = 0.0, deltaTotal = 0.0;
  for (size_t worker = 0; worker < workerCount; ++worker) {
    oldTotal += oldPartials[worker];
    newTotal += newPartials[worker];
    deltaTotal += deltaPartials[worker];
  }
  if (oldError) *oldError = oldTotal;
  if (newError) *newError = newTotal;
  return deltaTotal;
}

}  // namespace gtsam::timing::bal
