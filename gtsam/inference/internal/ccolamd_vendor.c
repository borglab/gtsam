/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/* Prefix the bundled C implementation so it can coexist with SuiteSparse. */
/* clang-format off */
#include "CCOLAMDSymbols.h"
#include "../../3rdparty/CCOLAMD/Source/ccolamd.c"
/* clang-format on */
