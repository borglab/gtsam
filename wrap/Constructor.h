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

#include <string>
#include <vector>

#include "Argument.h"

namespace wrap {

// Constructor class
struct Constructor {

	/// Constructor creates an empty class
	Constructor(bool verbose = false) :
			verbose_(verbose) {
	}

	// Then the instance variables are set directly by the Module constructor
    std::vector<ArgumentList> args_list;
	std::string name;
	bool verbose_;

	// MATLAB code generation
	// toolboxPath is main toolbox directory, e.g., ../matlab
	// classFile is class proxy file, e.g., ../matlab/@Point2/Point2.m

	/// wrapper name
	std::string matlab_wrapper_name(const std::string& className) const;

	/**
	 * Create fragment to select constructor in proxy class, e.g.,
	 * if nargin == 2, obj.self = new_Pose3_RP(varargin{1},varargin{2}); end
	 */
	void proxy_fragment(FileWriter& file, const std::string& wrapperName,
	        bool hasParent, const int id, const ArgumentList args) const;

	/// cpp wrapper
	std::string wrapper_fragment(FileWriter& file,
			 const std::string& cppClassName,
			 const std::string& matlabUniqueName,
			 const std::string& cppBaseClassName,
			 int id,
			 const ArgumentList& al) const;

	/// constructor function
	void generate_construct(FileWriter& file, const std::string& cppClassName,
	        std::vector<ArgumentList>& args_list) const;
	
};


} // \namespace wrap
