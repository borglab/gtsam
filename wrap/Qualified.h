/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Qualified.h
 * @brief Qualified name
 * @author Frank Dellaert
 * @date Nov 11, 2014
 **/

#pragma once

#include <string>
#include <vector>

namespace wrap {

/**
 * Class to encapuslate a qualified name, i.e., with (nested) namespaces
 */
struct Qualified {

  std::vector<std::string> namespaces; ///< Stack of namespaces
  std::string name; ///< type name

  Qualified(const std::string& name_ = "") :
      name(name_) {
  }

  bool empty() const {
    return namespaces.empty() && name.empty();
  }

  void clear() {
    namespaces.clear();
    name.clear();
  }

  bool operator!=(const Qualified& other) const {
    return other.name != name || other.namespaces != namespaces;
  }

  /// Return a qualified string using given delimiter
  std::string qualifiedName(const std::string& delimiter = "") const {
    std::string result;
    for (std::size_t i = 0; i < namespaces.size(); ++i)
      result += (namespaces[i] + delimiter);
    result += name;
    return result;
  }

  /// Return a matlab file name, i.e. "toolboxPath/+ns1/+ns2/name.m"
  std::string matlabName(const std::string& toolboxPath) const {
    std::string result = toolboxPath;
    for (std::size_t i = 0; i < namespaces.size(); ++i)
      result += ("/+" + namespaces[i]);
    result += "/" + name + ".m";
    return result;
  }

  friend std::ostream& operator<<(std::ostream& os, const Qualified& q) {
    os << q.qualifiedName("::");
    return os;
  }

};

} // \namespace wrap

