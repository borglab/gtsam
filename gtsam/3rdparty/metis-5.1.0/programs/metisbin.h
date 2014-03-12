/*
 * metisbin.h
 *
 * This file contains the various header inclusions
 *
 * Started 8/9/02
 * George
 */

#include <GKlib.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <limits.h>
#include <signal.h>
#include <setjmp.h>
#include <assert.h>


#if defined(ENABLE_OPENMP)
  #include <omp.h>
#endif


#include <metis.h>
#include "../libmetis/rename.h"
#include "../libmetis/gklib_defs.h"
#include "../libmetis/defs.h"
#include "../libmetis/struct.h"
#include "../libmetis/macros.h"
#include "../libmetis/proto.h"
#include "defs.h"
#include "struct.h"
#include "proto.h"


#if defined(COMPILER_GCC)
extern char* strdup (const char *);
#endif

#if defined(COMPILER_MSC)
#if defined(rint)
  #undef rint
#endif
#define rint(x) ((idx_t)((x)+0.5))  /* MSC does not have rint() function */
#define __func__ "dummy-function"
#endif
