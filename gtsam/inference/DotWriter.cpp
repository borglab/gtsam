/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   DotWriter.cpp
 * @brief  Graphviz formatting for factor graphs.
 * @author Frank Dellaert
 * @date December, 2021
 */

#include <gtsam/inference/DotWriter.h>

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>

#include <ostream>

using namespace std;

namespace gtsam {

void DotWriter::graphPreamble(ostream* os) const {
  *os << "graph {\n";
  *os << "  size=\"" << figureWidthInches << "," << figureHeightInches
      << "\";\n\n";
}

void DotWriter::digraphPreamble(ostream* os) const {
  *os << "digraph {\n";
  *os << "  size=\"" << figureWidthInches << "," << figureHeightInches
      << "\";\n\n";
}

void DotWriter::drawVariable(Key key, const KeyFormatter& keyFormatter,
                             const boost::optional<Vector2>& position,
                             ostream* os) const {
  // Label the node with the label from the KeyFormatter
  *os << "  var" << key << "[label=\"" << keyFormatter(key)
      << "\"";
  if (position) {
    *os << ", pos=\"" << position->x() << "," << position->y() << "!\"";
  }
  if (boxes.count(key)) {
    *os << ", shape=box";
  }
  *os << "];\n";
}

void DotWriter::DrawFactor(size_t i, const boost::optional<Vector2>& position,
                           ostream* os) {
  *os << "  factor" << i << "[label=\"\", shape=point";
  if (position) {
    *os << ", pos=\"" << position->x() << "," << position->y() << "!\"";
  }
  *os << "];\n";
}

static void ConnectVariables(Key key1, Key key2,
                             const KeyFormatter& keyFormatter, ostream* os) {
  *os << "  var" << key1 << "--"
      << "var" << key2 << ";\n";
}

static void ConnectVariableFactor(Key key, const KeyFormatter& keyFormatter,
                                  size_t i, ostream* os) {
  *os << "  var" << key << "--"
      << "factor" << i << ";\n";
}

/// Return variable position or none
boost::optional<Vector2> DotWriter::variablePos(Key key) const {
  boost::optional<Vector2> result = boost::none;

  // Check position hint
  Symbol symbol(key);
  auto hint = positionHints.find(symbol.chr());
  if (hint != positionHints.end())
    result.reset(Vector2(symbol.index(), hint->second));

  // Override with explicit position, if given.
  auto pos = variablePositions.find(key);
  if (pos != variablePositions.end())
    result.reset(pos->second);

  return result;
}

void DotWriter::processFactor(size_t i, const KeyVector& keys,
                              const KeyFormatter& keyFormatter,
                              const boost::optional<Vector2>& position,
                              ostream* os) const {
  if (plotFactorPoints) {
    if (binaryEdges && keys.size() == 2) {
      ConnectVariables(keys[0], keys[1], keyFormatter, os);
    } else {
      // Create dot for the factor.
      if (!position && factorPositions.count(i))
        DrawFactor(i, factorPositions.at(i), os);
      else
        DrawFactor(i, position, os);

      // Make factor-variable connections
      if (connectKeysToFactor) {
        for (Key key : keys) {
          ConnectVariableFactor(key, keyFormatter, i, os);
        }
      }
    }
  } else {
    // just connect variables in a clique
    for (Key key1 : keys) {
      for (Key key2 : keys) {
        if (key2 > key1) {
          ConnectVariables(key1, key2, keyFormatter, os);
        }
      }
    }
  }
}

}  // namespace gtsam
