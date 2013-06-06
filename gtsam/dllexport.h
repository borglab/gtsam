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

// Macros for exporting DLL symbols on Windows
// Usage example:
// In header file:
//   class GTSAM_EXPORT MyClass { ... };
//   
// Results in the following declarations:
// When included while compiling the GTSAM library itself:
//   class __declspec(dllexport) MyClass { ... };
// When included while compiling other code against GTSAM:
//   class __declspec(dllimport) MyClass { ... };
#ifdef _WIN32
#  ifdef GTSAM_EXPORTS
#    define GTSAM_EXPORT __declspec(dllexport)
#    define GTSAM_EXTERN_EXPORT __declspec(dllexport) extern
#  else
#    ifndef GTSAM_IMPORT_STATIC
#      define GTSAM_EXPORT __declspec(dllimport)
#      define GTSAM_EXTERN_EXPORT __declspec(dllimport)
#    else /* GTSAM_IMPORT_STATIC */
#      define GTSAM_EXPORT
#      define GTSAM_EXTERN_EXPORT extern
#    endif /* GTSAM_IMPORT_STATIC */
#  endif /* GTSAM_EXPORTS */
#else /* _WIN32 */
#  define GTSAM_EXPORT
#  define GTSAM_EXTERN_EXPORT extern
#endif
