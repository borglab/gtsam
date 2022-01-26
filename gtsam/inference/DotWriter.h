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
#include <map>

namespace gtsam {

/**
 * @brief DotWriter is a helper class for writing graphviz .dot files.
 * @addtogroup inference
 */
struct GTSAM_EXPORT DotWriter {
  double figureWidthInches;   ///< The figure width on paper in inches
  double figureHeightInches;  ///< The figure height on paper in inches
  bool plotFactorPoints;  ///< Plots each factor as a dot between the variables
  bool connectKeysToFactor;  ///< Draw a line from each key within a factor to
                             ///< the dot of the factor
  bool binaryEdges;          ///< just use non-dotted edges for binary factors

  /// (optional for each variable) Manually specify variable node positions
  std::map<gtsam::Key, Vector2> variablePositions;

  explicit DotWriter(double figureWidthInches = 5,
                     double figureHeightInches = 5,
                     bool plotFactorPoints = true,
                     bool connectKeysToFactor = true, bool binaryEdges = false)
      : figureWidthInches(figureWidthInches),
        figureHeightInches(figureHeightInches),
        plotFactorPoints(plotFactorPoints),
        connectKeysToFactor(connectKeysToFactor),
        binaryEdges(binaryEdges) {}

  /// Write out preamble for graph, including size.
  void graphPreamble(std::ostream* os) const;

  /// Write out preamble for digraph, including size.
  void digraphPreamble(std::ostream* os) const;

  /// Create a variable dot fragment.
  static void DrawVariable(Key key, const KeyFormatter& keyFormatter,
                           const boost::optional<Vector2>& position,
                           std::ostream* os);

  /// Create factor dot.
  static void DrawFactor(size_t i, const boost::optional<Vector2>& position,
                         std::ostream* os);

  /// Return variable position or none
  boost::optional<Vector2> variablePos(Key key) const {
    auto it = variablePositions.find(key);
    if (it == variablePositions.end())
      return boost::none;
    else
      return it->second;
  }

  /// Draw a single factor, specified by its index i and its variable keys.
  void processFactor(size_t i, const KeyVector& keys,
                     const KeyFormatter& keyFormatter,
                     const boost::optional<Vector2>& position,
                     std::ostream* os) const;
};

}  // namespace gtsam
