/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DotWriter.h
 * @brief Graphviz formatter
 * @author Frank Dellaert
 * @date December, 2021
 */

#pragma once

#include <gtsam/base/FastVector.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Key.h>

#include <iosfwd>

namespace gtsam {

/// Graphviz formatter.
struct GTSAM_EXPORT DotWriter {
  double figureWidthInches;   ///< The figure width on paper in inches
  double figureHeightInches;  ///< The figure height on paper in inches
  bool plotFactorPoints;  ///< Plots each factor as a dot between the variables
  bool connectKeysToFactor;  ///< Draw a line from each key within a factor to
                             ///< the dot of the factor
  bool binaryEdges;          ///< just use non-dotted edges for binary factors

  DotWriter()
      : figureWidthInches(5),
        figureHeightInches(5),
        plotFactorPoints(true),
        connectKeysToFactor(true),
        binaryEdges(true) {}

  /// Write out preamble, including size.
  void writePreamble(std::ostream* os) const;

  /// Create a variable dot fragment.
  static void DrawVariable(Key key, const KeyFormatter& keyFormatter,
                           const boost::optional<Vector2>& position,
                           std::ostream* os);

  /// Create factor dot.
  static void DrawFactor(size_t i, const boost::optional<Vector2>& position,
                         std::ostream* os);

  /// Connect two variables.
  static void ConnectVariables(Key key1, Key key2, std::ostream* os);

  /// Connect variable and factor.
  static void ConnectVariableFactor(Key key, size_t i, std::ostream* os);

  /// Draw a single factor, specified by its index i and its variable keys.
  void processFactor(size_t i, const KeyVector& keys,
                     const boost::optional<Vector2>& position,
                     std::ostream* os) const;
};

}  // namespace gtsam
