/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    lapack.h
 * @brief   Handles wrapping of LAPACK functions that we use
 * @author  Richard Roberts
 * @created Nov 6, 2010
 */

#pragma once

// Prototypes of LAPACK functions that we use
extern "C" {

#include <cblas.h>

/* Subroutine */ int dpotrf_(char *uplo, int *n, double *a, int *lda,
    int *info);

inline int lapack_dpotrf(char uplo, int n, double* a, int lda) {
  int info;
  dpotrf_(&uplo, &n, a, &lda, &info);
  return info;
}

}

