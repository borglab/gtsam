/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearOptimizer.h
 * @brief   Common Interface for Linear Optimizers
 * @author  Fan Jiang
 */

#pragma once

#include "gtsam/nonlinear/NonlinearOptimizerParams.h"

class LinearOptimizer {
public:
    LinearOptimizer();
    LinearOptimizer(gtsam::NonlinearOptimizerParams param);

    LinearOptimizer(LinearOptimizer&) = delete;
};

