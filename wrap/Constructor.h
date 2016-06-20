/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Constructor.h
 * @brief class describing a constructor + code generation
 * @author Frank Dellaert
 * @author Richard Roberts
 **/

#pragma once

#include "OverloadedFunction.h"
#include <boost/optional.hpp>
#include <string>
#include <vector>

namespace wrap {

// Constructor class
struct Constructor: public OverloadedFunction {

  typedef const std::string& Str;

  /// Constructor creates an empty class
  Constructor(bool verbose = false) {
    verbose_ = verbose;
  }

  Constructor expandTemplate(const TemplateSubstitution& ts) const {
    Constructor inst = *this;
    inst.argLists_ = expandArgumentListsTemplate(ts);
    inst.name_ = ts.expandedClassName();
    return inst;
  }

  // MATLAB code generation
  // toolboxPath is main toolbox directory, e.g., ../matlab
  // classFile is class proxy file, e.g., ../matlab/@Point2/Point2.m

  /// wrapper name
  std::string matlab_wrapper_name(Str className) const;

  void comment_fragment(FileWriter& proxyFile) const {
    if (nrOverloads() > 0)
      proxyFile.oss << "%\n%-------Constructors-------\n";
    for (size_t i = 0; i < nrOverloads(); i++) {
      proxyFile.oss << "%";
      argumentList(i).emit_prototype(proxyFile, name_);
      proxyFile.oss << "\n";
    }
  }

  /**
   * Create fragment to select constructor in proxy class, e.g.,
   * if nargin == 2, obj.self = new_Pose3_RP(varargin{1},varargin{2}); end
   */
  void proxy_fragment(FileWriter& file, Str wrapperName, bool hasParent,
      const int id, const ArgumentList args) const;

  /// cpp wrapper
  std::string wrapper_fragment(FileWriter& file, Str cppClassName,
      Str matlabUniqueName, boost::optional<std::string> cppBaseClassName, int id,
      const ArgumentList& al) const;

  /// constructor function
  void generate_construct(FileWriter& file, Str cppClassName,
      std::vector<ArgumentList>& args_list) const;

  // emit python wrapper
  void python_wrapper(FileWriter& wrapperFile, Str className) const;

  friend std::ostream& operator<<(std::ostream& os, const Constructor& m) {
    for (size_t i = 0; i < m.nrOverloads(); i++)
      os << m.name_ << m.argLists_[i];
    return os;
  }

};

} // \namespace wrap
