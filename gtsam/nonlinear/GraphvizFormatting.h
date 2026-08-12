/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GraphvizFormatting.h
 * @brief Graphviz formatter for NonlinearFactorGraph
 * @author Frank Dellaert
 * @date December, 2021
 */

#pragma once

#include <gtsam/inference/DotWriter.h>

namespace gtsam {

class Values;
class Value;

/**
 * Formatting options and functions for saving a NonlinearFactorGraph instance
 * in GraphViz format.
 */
struct GTSAM_EXPORT GraphvizFormatting : public DotWriter {
  /// World axes to be assigned to paper axes
  enum Axis { X, Y, Z, NEGX, NEGY, NEGZ };

  Axis paperHorizontalAxis;  ///< The world axis assigned to the horizontal
                             ///< paper axis
  Axis paperVerticalAxis;    ///< The world axis assigned to the vertical paper
                             ///< axis
  double scale;  ///< Scale all positions to reduce / increase density
  bool mergeSimilarFactors;  ///< Merge multiple factors that have the same
                             ///< connectivity

  /// Default constructor sets up robot coordinates.  Paper horizontal is robot
  /// Y, paper vertical is robot X.  Default figure size of 5x5 in.
  GraphvizFormatting()
      : paperHorizontalAxis(Y),
        paperVerticalAxis(X),
        scale(1),
        mergeSimilarFactors(false) {}

  // Find bounds
  Vector2 findBounds(const Values& values, const KeySet& keys) const;

  /// Extract a Vector2 from either Vector2, Pose2, Pose3, or Point3
  std::optional<Vector2> extractPosition(const Value& value) const;

  /// Return affinely transformed variable position if it exists.
  std::optional<Vector2> variablePos(const Values& values, const Vector2& min,
                                       Key key) const;

  /// Return affinely transformed factor position if it exists.
  std::optional<Vector2> factorPos(const Vector2& min, size_t i) const;
};

}  // namespace gtsam
