/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
  * @file    precompiled_header.h>
  * @brief   Include headers that will be included nearly everywhere
  * @author  Frank Dellaert
  * @date    November 2018
  */

#pragma once

// All headers in base, except:
// treeTraversal-inst.h: very specific to only a few compilation units
// numericalDerivative.h : includes things in linear, nonlinear :-(
// testLie.h: includes numericalDerivative
#include <gtsam/base/Lie.h>
#include <gtsam/base/chartTesting.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/base/concepts.h>
#include <gtsam/base/ConcurrentMap.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/DSFVector.h>
#include <gtsam/base/FastDefaultAllocator.h>
#include <gtsam/base/FastList.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Group.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/base/serialization.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/types.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/VerticalBlockMatrix.h>


