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
 **/

#pragma once

#include <string>
#include <vector>

#include "Argument.h"

namespace wrap {

// Constructor class
struct Constructor {

	/// Constructor creates an empty class
	Constructor(bool verbose = true) :
			verbose_(verbose) {
	}

	// Then the instance variables are set directly by the Module constructor
	ArgumentList args;
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
	void matlab_proxy_fragment(FileWriter& file, const std::string& className) const;

	/// m-file
	void matlab_mfile(const std::string& toolboxPath,
			const std::string& qualifiedMatlabName) const;

	/// cpp wrapper
	void matlab_wrapper(const std::string& toolboxPath,
			 const std::string& cppClassName,
			 const std::string& matlabClassName,
			 const std::vector<std::string>& using_namespaces,
			 const std::vector<std::string>& includes) const;
};

} // \namespace wrap

