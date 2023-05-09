/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     global_includes.h
 * @brief    Included from all GTSAM files
 * @author   Richard Roberts
 * @addtogroup base
 */

#pragma once

#include <gtsam/config.h>      // Configuration from CMake
#include <gtsam/base/types.h>  // Basic types, constants, and compatibility functions
// types.h includes dllexport.h, which contains macros for dllspec tags for Windows DLLs
