/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmTypes.h
 * @brief   Trivially copyable measurement and noise types for CUDA SFM
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

namespace gtsam::cuda {

struct SfmObservation {
  int cameraSlot;
  int pointSlot;
  double measuredU;
  double measuredV;
};

struct SfmSqrtInfo2 {
  double r00;
  double r01;
  double r11;
};

enum class SfmRobustModelKind {
  None,
  Huber,
  Tukey,
};

enum class SfmRobustReweightScheme {
  Scalar,
  Block,
};

struct SfmRobustModel {
  SfmRobustModelKind kind;
  SfmRobustReweightScheme reweightScheme;
  double parameter;
};

}  // namespace gtsam::cuda
