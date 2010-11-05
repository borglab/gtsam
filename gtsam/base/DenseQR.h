/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DenseQR.h
 *
 *   Created on: Oct 19, 2010
 *       Author: nikai
 *  Description: Dense QR, inspired by Tim Davis's dense solver
 */

#pragma once

namespace gtsam {
	void DenseQR( int m, int n, int numPivotColumns,  // inputs
			double *F, int *Stair, double *W);            // outputs
}
