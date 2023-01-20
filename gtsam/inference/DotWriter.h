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
#include <set>

namespace gtsam {

/**
 * @brief DotWriter is a helper class for writing graphviz .dot files.
 * @ingroup inference
 */
struct GTSAM_EXPORT DotWriter {
  double figureWidthInches;   ///< The figure width on paper in inches
  double figureHeightInches;  ///< The figure height on paper in inches
  bool plotFactorPoints;  ///< Plots each factor as a dot between the variables
  bool connectKeysToFactor;  ///< Draw a line from each key within a factor to
                             ///< the dot of the factor
  bool binaryEdges;          ///< just use non-dotted edges for binary factors

  /**
   * Variable positions can be optionally specified and will be included in the
   * dot file with a "!' sign, so "neato" can use it to render them.
   */
  std::map<Key, Vector2> variablePositions;

  /**
   * The position hints allow one to use symbol character and index to specify
   * position. Unless variable positions are specified, if a hint is present for
   * a given symbol, it will be used to calculate the positions as (index,hint).
   */
  std::map<char, double> positionHints;

  /** A set of keys that will be displayed as a box */
  std::set<Key> boxes;

  /**
   * Factor positions can be optionally specified and will be included in the
   * dot file with a "!' sign, so "neato" can use it to render them.
   */
  std::map<size_t, Vector2> factorPositions;

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
  void drawVariable(Key key, const KeyFormatter& keyFormatter,
                    const boost::optional<Vector2>& position,
                    std::ostream* os) const;

  /// Create factor dot.
  static void DrawFactor(size_t i, const boost::optional<Vector2>& position,
                         std::ostream* os);

  /// Return variable position or none
  boost::optional<Vector2> variablePos(Key key) const;

  /// Draw a single factor, specified by its index i and its variable keys.
  void processFactor(size_t i, const KeyVector& keys,
                     const KeyFormatter& keyFormatter,
                     const boost::optional<Vector2>& position,
                     std::ostream* os) const;
};

}  // namespace gtsam
