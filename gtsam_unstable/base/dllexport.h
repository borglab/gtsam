/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     dllexport.h
 * @brief    Symbols for exporting classes and methods from DLLs
 * @author   Richard Roberts
 * @date     Mar 9, 2013
 */

#ifdef _WIN32
#  ifdef GTSAM_UNSTABLE_EXPORTS
#    define GTSAM_UNSTABLE_EXPORT __declspec(dllexport)
#    define GTSAM_UNSTABLE_EXTERN_EXPORT __declspec(dllexport) extern
#  else
#    ifndef GTSAM_UNSTABLE_IMPORT_STATIC
#      define GTSAM_UNSTABLE_EXPORT __declspec(dllimport)
#      define GTSAM_UNSTABLE_EXTERN_EXPORT __declspec(dllimport)
#    else /* GTSAM_UNSTABLE_IMPORT_STATIC */
#      define GTSAM_UNSTABLE_EXPORT
#      define GTSAM_UNSTABLE_EXTERN_EXPORT extern
#    endif /* GTSAM_UNSTABLE_IMPORT_STATIC */
#  endif /* GTSAM_UNSTABLE_EXPORTS */
#else /* _WIN32 */
#  define GTSAM_UNSTABLE_EXPORT
#  define GTSAM_UNSTABLE_EXTERN_EXPORT extern
#endif

