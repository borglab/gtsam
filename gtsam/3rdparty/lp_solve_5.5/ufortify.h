/*
 * FILE:
 *   ufortify.h
 *
 * DESCRIPTION:
 *   User options for fortify. Changes to this file require fortify.c to be
 * recompiled, but nothing else.
 */

#ifndef __UFORTIFY_H__
#define __UFORTIFY_H__

#define FORTIFY_STORAGE

#if defined MSDOS || defined __BORLANDC__ || defined __HIGHC__
# define KNOWS_POINTER_TYPE
#endif

#define FORTIFY_WAIT_FOR_KEY         /* Pause after message            */

#if !defined FORTIFY_BEFORE_SIZE
# define FORTIFY_BEFORE_SIZE     16  /* Bytes to allocate before block */
#endif

#if !defined FORTIFY_BEFORE_VALUE
# define FORTIFY_BEFORE_VALUE  0xA3  /* Fill value before block        */
#endif

#if !defined FORTIFY_AFTER_SIZE
# define FORTIFY_AFTER_SIZE      16  /* Bytes to allocate after block  */
#endif

#if !defined FORTIFY_AFTER_VALUE
# define FORTIFY_AFTER_VALUE   0xA5  /* Fill value after block         */
#endif

#define FILL_ON_MALLOC               /* Nuke out malloc'd memory       */

#if !defined FILL_ON_MALLOC_VALUE
# define FILL_ON_MALLOC_VALUE  0xA7  /* Value to initialize with       */
#endif

#define FILL_ON_FREE                 /* free'd memory is cleared       */

#if !defined FILL_ON_FREE_VALUE
# define FILL_ON_FREE_VALUE    0xA9  /* Value to de-initialize with    */
#endif

#define FORTIFY_CheckInterval 1      /* seconds */
/* #define CHECK_ALL_MEMORY_ON_MALLOC */
#define CHECK_ALL_MEMORY_ON_FREE
#define PARANOID_FREE

#define WARN_ON_MALLOC_FAIL    /* A debug is issued on a failed malloc */
#define WARN_ON_ZERO_MALLOC    /* A debug is issued on a malloc(0)     */
#define WARN_ON_FALSE_FAIL     /* See Fortify_SetMallocFailRate        */
#define WARN_ON_SIZE_T_OVERFLOW/* Watch for breaking the 64K limit in  */
                               /* some braindead architectures...      */

#define FORTIFY_LOCK()
#define FORTIFY_UNLOCK()

#endif
