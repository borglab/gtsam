/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/* Prefix every public symbol declared by the bundled CCOLAMD header. */
#define ccolamd_recommended gtsam_ccolamd_recommended
#define ccolamd_l_recommended gtsam_ccolamd_l_recommended
#define ccolamd_set_defaults gtsam_ccolamd_set_defaults
#define ccolamd_l_set_defaults gtsam_ccolamd_l_set_defaults
#define ccolamd gtsam_ccolamd
#define ccolamd_l gtsam_ccolamd_l
#define csymamd gtsam_csymamd
#define csymamd_l gtsam_csymamd_l
#define ccolamd_report gtsam_ccolamd_report
#define ccolamd_l_report gtsam_ccolamd_l_report
#define csymamd_report gtsam_csymamd_report
#define csymamd_l_report gtsam_csymamd_l_report
#define ccolamd2 gtsam_ccolamd2
#define ccolamd2_l gtsam_ccolamd2_l
#define ccolamd_apply_order gtsam_ccolamd_apply_order
#define ccolamd_l_apply_order gtsam_ccolamd_l_apply_order
#define ccolamd_fsize gtsam_ccolamd_fsize
#define ccolamd_l_fsize gtsam_ccolamd_l_fsize
#define ccolamd_postorder gtsam_ccolamd_postorder
#define ccolamd_l_postorder gtsam_ccolamd_l_postorder
#define ccolamd_post_tree gtsam_ccolamd_post_tree
#define ccolamd_l_post_tree gtsam_ccolamd_l_post_tree

/* Prefix the SuiteSparse_config state and its complete public C API. */
#define SuiteSparse_config_struct gtsam_SuiteSparse_config_struct
#define SuiteSparse_config gtsam_SuiteSparse_config
#define SuiteSparse_start gtsam_SuiteSparse_start
#define SuiteSparse_finish gtsam_SuiteSparse_finish
#define SuiteSparse_malloc gtsam_SuiteSparse_malloc
#define SuiteSparse_calloc gtsam_SuiteSparse_calloc
#define SuiteSparse_realloc gtsam_SuiteSparse_realloc
#define SuiteSparse_free gtsam_SuiteSparse_free
#define SuiteSparse_tic gtsam_SuiteSparse_tic
#define SuiteSparse_toc gtsam_SuiteSparse_toc
#define SuiteSparse_time gtsam_SuiteSparse_time
#define SuiteSparse_hypot gtsam_SuiteSparse_hypot
#define SuiteSparse_divcomplex gtsam_SuiteSparse_divcomplex
#define SuiteSparse_version gtsam_SuiteSparse_version
