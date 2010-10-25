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
