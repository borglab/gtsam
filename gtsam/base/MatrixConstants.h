/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    MatrixConstants.h
 * @brief   Macros for Matrix constants to avoid excessive template
 * instantiation. Avoid using in headers to prevent name pollution.
 * @author  Gold856
 */

#pragma once

#define I_1x1 Matrix1::Identity()
#define Z_1x1 Matrix1::Constant(0.0)

#define I_2x2 Matrix2::Identity()
#define Z_2x2 Matrix2::Constant(0.0)

#define I_3x3 Matrix3::Identity()
#define Z_3x3 Matrix3::Constant(0.0)

#define I_4x4 Matrix4::Identity()
#define Z_4x4 Matrix4::Constant(0.0)

#define I_5x5 Matrix5::Identity()
#define Z_5x5 Matrix5::Constant(0.0)

#define I_6x6 Matrix6::Identity()
#define Z_6x6 Matrix6::Constant(0.0)

#define I_7x7 Matrix7::Identity()
#define Z_7x7 Matrix7::Constant(0.0)

#define I_8x8 Matrix8::Identity()
#define Z_8x8 Matrix8::Constant(0.0)

#define I_9x9 Matrix9::Identity()
#define Z_9x9 Matrix9::Constant(0.0)
