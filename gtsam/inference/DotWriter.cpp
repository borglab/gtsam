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

#include <gtsam/base/Vector.h>
#include <gtsam/inference/DotWriter.h>

#include <ostream>

using namespace std;

namespace gtsam {

void DotWriter::writePreamble(ostream* os) const {
  *os << "graph {\n";
  *os << "  size=\"" << figureWidthInches << "," << figureHeightInches
      << "\";\n\n";
}

void DotWriter::DrawVariable(Key key, const KeyFormatter& keyFormatter,
                             const boost::optional<Vector2>& position,
                             ostream* os) {
  // Label the node with the label from the KeyFormatter
  *os << "  var" << keyFormatter(key) << "[label=\"" << keyFormatter(key)
      << "\"";
  if (position) {
    *os << ", pos=\"" << position->x() << "," << position->y() << "!\"";
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
                                 const KeyFormatter& keyFormatter,
                                 ostream* os) {
  *os << "  var" << keyFormatter(key1) << "--"
      << "var" << keyFormatter(key2) << ";\n";
}

static void ConnectVariableFactor(Key key, const KeyFormatter& keyFormatter,
                                      size_t i, ostream* os) {
  *os << "  var" << keyFormatter(key) << "--"
      << "factor" << i << ";\n";
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
