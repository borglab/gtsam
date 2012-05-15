/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * parseUAI.h
 * @brief: parse UAI 2008 format
 * @date March 5, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <string>
#include <gtsam/discrete/TypedDiscreteFactorGraph.h>

namespace gtsam {

	/**
	 * Constructor from file
	 * For now assumes in .uai format from UAI'08 Probablistic Inference Evaluation
	 * See http://graphmod.ics.uci.edu/uai08/FileFormat
	 */
	bool parseUAI(const std::string& filename,
			gtsam::TypedDiscreteFactorGraph& graph);

} // gtsam
