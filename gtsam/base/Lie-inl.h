/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Lie-inl.h
 * @date Jan 9, 2010
 * @author Richard Roberts
 * @brief Instantiate macro for Lie type
 */

#pragma once

#include <gtsam/base/Lie.h>

#define INSTANTIATE_LIE(T) \
  template T between_default(const T&, const T&); \
  template Vector logmap_default(const T&, const T&); \
  template T expmap_default(const T&, const Vector&);


