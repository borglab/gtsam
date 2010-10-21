/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DenseQRUtil.h
 *
 *   Created on: Jul 1, 2010
 *       Author: nikai
 *  Description: the utility functions for DenseQR
 */

#pragma once

#include <gtsam/base/Matrix.h>

#ifdef GT_USE_LAPACK
#include <gtsam/base/DenseQR.h>

namespace gtsam {

	/** make stairs and speed up householder_denseqr. Stair is defined as the row index of where zero entries start in each column */
	long* MakeStairs(Matrix &A);

	/** Householder tranformation, zeros below diagonal */
	void householder_denseqr(Matrix &A, long* Stair = NULL);

    /** Householder tranformation in column mafor form */ 
	void householder_denseqr_colmajor(boost::numeric::ublas::matrix<double, boost::numeric::ublas::column_major>& A, long *Stair);
}
#endif
