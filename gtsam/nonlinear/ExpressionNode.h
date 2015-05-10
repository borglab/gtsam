/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ExpressionNode.h
 * @date May 10, 2015
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief ExpressionNode class
 */

#pragma once

#include <ostream>
#include <map>


namespace gtsam {

/**
 * Expression node. The superclass for objects that do the heavy lifting
 * An Expression<T> has a pointer to an ExpressionNode<T> underneath
 * allowing Expressions to have polymorphic behaviour even though they
 * are passed by value. This is the same way boost::function works.
 * http://loki-lib.sourceforge.net/html/a00652.html
 */
template<class T>
class ExpressionNode {

protected:

  size_t traceSize_;

  /// Constructor, traceSize is size of the execution trace of expression rooted here
  ExpressionNode(size_t traceSize = 0) :
      traceSize_(traceSize) {
  }

public:

  /// Destructor
  virtual ~ExpressionNode() {
  }

  /// Streaming
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os,
      const ExpressionNode& node) {
    os << "Expression of type " << typeid(T).name();
    if (node.traceSize_ > 0)
      os << ", trace size = " << node.traceSize_;
    os << "\n";
    return os;
  }

  /// Return keys that play in this expression as a set
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    return keys;
  }

  /// Return dimensions for each argument, as a map
  virtual void dims(std::map<Key, int>& map) const {
  }

  // Return size needed for memory buffer in traceExecution
  size_t traceSize() const {
    return traceSize_;
  }

  /// Return value
  virtual T value(const Values& values) const = 0;

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* traceStorage) const = 0;
};

}
