/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GraphvizFormatting.cpp
 * @brief Graphviz formatter for NonlinearFactorGraph
 * @author Frank Dellaert
 * @date December, 2021
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GraphvizFormatting.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace gtsam {

Vector2 GraphvizFormatting::findBounds(const Values& values,
                                       const KeySet& keys) const {
  Vector2 min;
  min.x() = std::numeric_limits<double>::infinity();
  min.y() = std::numeric_limits<double>::infinity();
  for (const Key& key : keys) {
    if (values.exists(key)) {
      std::optional<Vector2> xy = extractPosition(values.at(key));
      if (xy) {
        if (xy->x() < min.x()) min.x() = xy->x();
        if (xy->y() < min.y()) min.y() = xy->y();
      }
    }
  }
  return min;
}

std::optional<Vector2> GraphvizFormatting::extractPosition(
    const Value& value) const {
  Vector3 t;
  if (const GenericValue<Pose2>* p =
          dynamic_cast<const GenericValue<Pose2>*>(&value)) {
    t << p->value().x(), p->value().y(), 0;
  } else if (const GenericValue<Vector2>* p =
                 dynamic_cast<const GenericValue<Vector2>*>(&value)) {
    t << p->value().x(), p->value().y(), 0;
  } else if (const GenericValue<Vector>* p =
                 dynamic_cast<const GenericValue<Vector>*>(&value)) {
    if (p->dim() == 2) {
      const Eigen::Ref<const Vector2> p_2d(p->value());
      t << p_2d.x(), p_2d.y(), 0;
    } else if (p->dim() == 3) {
      const Eigen::Ref<const Vector3> p_3d(p->value());
      t = p_3d;
    } else {
      return {};
    }
  } else if (const GenericValue<Pose3>* p =
                 dynamic_cast<const GenericValue<Pose3>*>(&value)) {
    t = p->value().translation();
  } else if (const GenericValue<Point3>* p =
                 dynamic_cast<const GenericValue<Point3>*>(&value)) {
    t = p->value();
  } else {
    return {};
  }
  double x, y;
  switch (paperHorizontalAxis) {
    case X:
      x = t.x();
      break;
    case Y:
      x = t.y();
      break;
    case Z:
      x = t.z();
      break;
    case NEGX:
      x = -t.x();
      break;
    case NEGY:
      x = -t.y();
      break;
    case NEGZ:
      x = -t.z();
      break;
    default:
      throw std::runtime_error("Invalid enum value");
  }
  switch (paperVerticalAxis) {
    case X:
      y = t.x();
      break;
    case Y:
      y = t.y();
      break;
    case Z:
      y = t.z();
      break;
    case NEGX:
      y = -t.x();
      break;
    case NEGY:
      y = -t.y();
      break;
    case NEGZ:
      y = -t.z();
      break;
    default:
      throw std::runtime_error("Invalid enum value");
  }
  return Vector2(x, y);
}

std::optional<Vector2> GraphvizFormatting::variablePos(const Values& values,
                                                         const Vector2& min,
                                                         Key key) const {
  if (!values.exists(key)) return DotWriter::variablePos(key);
  std::optional<Vector2> xy = extractPosition(values.at(key));
  if (xy) {
    xy->x() = scale * (xy->x() - min.x());
    xy->y() = scale * (xy->y() - min.y());
  }
  return xy;
}

std::optional<Vector2> GraphvizFormatting::factorPos(const Vector2& min,
                                                       size_t i) const {
  if (factorPositions.size() == 0) return {};
  auto it = factorPositions.find(i);
  if (it == factorPositions.end()) return {};
  auto pos = it->second;
  return Vector2(scale * (pos.x() - min.x()), scale * (pos.y() - min.y()));
}

/* ************************************************************************* */
void NonlinearFactorGraph::dot(std::ostream& os, const Values& values,
                               const KeyFormatter& keyFormatter,
                               const GraphvizFormatting& writer) const {
  writer.graphPreamble(&os);

  // Find bounds (imperative)
  KeySet keys = this->keys();
  Vector2 min = writer.findBounds(values, keys);

  // Create nodes for each variable in the graph
  for (Key key : keys) {
    auto position = writer.variablePos(values, min, key);
    writer.drawVariable(key, keyFormatter, position, &os);
  }
  os << "\n";

  if (writer.mergeSimilarFactors) {
    // Remove duplicate factors
    std::set<KeyVector> structure;
    for (const sharedFactor& factor : factors_) {
      if (factor) {
        KeyVector factorKeys = factor->keys();
        std::sort(factorKeys.begin(), factorKeys.end());
        structure.insert(factorKeys);
      }
    }

    // Create factors and variable connections
    size_t i = 0;
    for (const KeyVector& factorKeys : structure) {
      writer.processFactor(i++, factorKeys, keyFormatter, {}, &os);
    }
  } else {
    // Create factors and variable connections
    for (size_t i = 0; i < size(); ++i) {
      const NonlinearFactor::shared_ptr& factor = at(i);
      if (factor) {
        const KeyVector& factorKeys = factor->keys();
        writer.processFactor(i, factorKeys, keyFormatter,
                             writer.factorPos(min, i), &os);
      }
    }
  }

  os << "}\n";
  std::flush(os);
}

/* ************************************************************************* */
std::string NonlinearFactorGraph::dot(const Values& values,
                                      const KeyFormatter& keyFormatter,
                                      const GraphvizFormatting& writer) const {
  std::stringstream ss;
  dot(ss, values, keyFormatter, writer);
  return ss.str();
}

/* ************************************************************************* */
void NonlinearFactorGraph::saveGraph(const std::string& filename,
                                     const Values& values,
                                     const KeyFormatter& keyFormatter,
                                     const GraphvizFormatting& writer) const {
  std::ofstream of(filename);
  dot(of, values, keyFormatter, writer);
  of.close();
}

}  // namespace gtsam
