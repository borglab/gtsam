/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    svdcmp.h
 * @brief   SVD decomposition adapted from NRC
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

/** SVD decomposition */
void svdcmp(double **a, int m, int n, double w[], double **v, bool sort = true);

