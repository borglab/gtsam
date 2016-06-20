/*!
\file  memory.c
\brief This file contains various allocation routines 

The allocation routines included are for 1D and 2D arrays of the 
most datatypes that GKlib support. Many of these routines are 
defined with the help of the macros in gk_memory.h. These macros 
can be used to define other memory allocation routines.

\date   Started 4/3/2007
\author George
\version\verbatim $Id: memory.c 10783 2011-09-21 23:19:56Z karypis $ \endverbatim
*/


#include <GKlib.h>

/* This is for the global mcore that tracks all heap allocations */
static __thread gk_mcore_t *gkmcore = NULL;


/*************************************************************************/
/*! Define the set of memory allocation routines for each data type */
/**************************************************************************/
GK_MKALLOC(gk_c,   char)
GK_MKALLOC(gk_i,   int)
GK_MKALLOC(gk_i32, int32_t)
GK_MKALLOC(gk_i64, int64_t)
GK_MKALLOC(gk_z,   ssize_t)
GK_MKALLOC(gk_f,   float)
GK_MKALLOC(gk_d,   double)
GK_MKALLOC(gk_idx, gk_idx_t)

GK_MKALLOC(gk_ckv,   gk_ckv_t)
GK_MKALLOC(gk_ikv,   gk_ikv_t)
GK_MKALLOC(gk_i32kv, gk_i32kv_t)
GK_MKALLOC(gk_i64kv, gk_i64kv_t)
GK_MKALLOC(gk_zkv,   gk_zkv_t)
GK_MKALLOC(gk_fkv,   gk_fkv_t)
GK_MKALLOC(gk_dkv,   gk_dkv_t)
GK_MKALLOC(gk_skv,   gk_skv_t)
GK_MKALLOC(gk_idxkv, gk_idxkv_t)






/*************************************************************************/
/*! This function allocates a two-dimensional matrix.
  */
/*************************************************************************/
void gk_AllocMatrix(void ***r_matrix, size_t elmlen, size_t ndim1, size_t ndim2)
{
  gk_idx_t i, j;
  void **matrix;

  *r_matrix = NULL;

  if ((matrix = (void **)gk_malloc(ndim1*sizeof(void *), "gk_AllocMatrix: matrix")) == NULL)
    return;

  for (i=0; i<ndim1; i++) {
    if ((matrix[i] = (void *)gk_malloc(ndim2*elmlen, "gk_AllocMatrix: matrix[i]")) == NULL) {
      for (j=0; j<i; j++) 
        gk_free((void **)&matrix[j], LTERM);
      return;
    }
  }

  *r_matrix = matrix;
}


/*************************************************************************/
/*! This function frees a two-dimensional matrix.
  */
/*************************************************************************/
void gk_FreeMatrix(void ***r_matrix, size_t ndim1, size_t ndim2)
{
  gk_idx_t i;
  void **matrix;

  if ((matrix = *r_matrix) == NULL)
    return;

  for (i=0; i<ndim1; i++) 
    gk_free((void **)&matrix[i], LTERM);

  gk_free((void **)r_matrix, LTERM); 

}


/*************************************************************************/
/*! This function initializes tracking of heap allocations. 
*/
/*************************************************************************/
int gk_malloc_init()
{
  if (gkmcore == NULL)
    gkmcore = gk_gkmcoreCreate();

  if (gkmcore == NULL)
    return 0;

  gk_gkmcorePush(gkmcore);

  return 1;
}


/*************************************************************************/
/*! This function frees the memory that has been allocated since the
    last call to gk_malloc_init().
*/
/*************************************************************************/
void gk_malloc_cleanup(int showstats)
{
  if (gkmcore != NULL) {
    gk_gkmcorePop(gkmcore);
    if (gkmcore->cmop == 0) {
      gk_gkmcoreDestroy(&gkmcore, showstats);
      gkmcore = NULL;
    }
  }
}


/*************************************************************************/
/*! This function is my wrapper around malloc that provides the following
    enhancements over malloc:
    * It always allocates one byte of memory, even if 0 bytes are requested.
      This is to ensure that checks of returned values do not lead to NULL
      due to 0 bytes requested.
    * It zeros-out the memory that is allocated. This is for a quick init
      of the underlying datastructures.
*/
/**************************************************************************/
void *gk_malloc(size_t nbytes, char *msg)
{
  void *ptr=NULL;

  if (nbytes == 0)
    nbytes++;  /* Force mallocs to actually allocate some memory */

  ptr = (void *)malloc(nbytes);

  if (ptr == NULL) {
    fprintf(stderr, "   Current memory used:  %10zu bytes\n", gk_GetCurMemoryUsed());
    fprintf(stderr, "   Maximum memory used:  %10zu bytes\n", gk_GetMaxMemoryUsed());
    gk_errexit(SIGMEM, "***Memory allocation failed for %s. Requested size: %zu bytes", 
        msg, nbytes);
    return NULL;
  }

  /* add this memory allocation */
  if (gkmcore != NULL) gk_gkmcoreAdd(gkmcore, GK_MOPT_HEAP, nbytes, ptr);

  /* zero-out the allocated space */
#ifndef NDEBUG
  memset(ptr, 0, nbytes);
#endif

  return ptr;
}


/*************************************************************************
* This function is my wrapper around realloc
**************************************************************************/
void *gk_realloc(void *oldptr, size_t nbytes, char *msg)
{
  void *ptr=NULL;

  if (nbytes == 0)
    nbytes++;  /* Force mallocs to actually allocate some memory */

  /* remove this memory de-allocation */
  if (gkmcore != NULL && oldptr != NULL) gk_gkmcoreDel(gkmcore, oldptr);

  ptr = (void *)realloc(oldptr, nbytes);

  if (ptr == NULL) {
    fprintf(stderr, "   Maximum memory used: %10zu bytes\n", gk_GetMaxMemoryUsed());
    fprintf(stderr, "   Current memory used: %10zu bytes\n", gk_GetCurMemoryUsed());
    gk_errexit(SIGMEM, "***Memory realloc failed for %s. " "Requested size: %zu bytes", 
        msg, nbytes);
    return NULL;
  }

  /* add this memory allocation */
  if (gkmcore != NULL) gk_gkmcoreAdd(gkmcore, GK_MOPT_HEAP, nbytes, ptr);

  return ptr;
}


/*************************************************************************
* This function is my wrapper around free, allows multiple pointers    
**************************************************************************/
void gk_free(void **ptr1,...)
{
  va_list plist;
  void **ptr;

  if (*ptr1 != NULL) {
    free(*ptr1);

    /* remove this memory de-allocation */
    if (gkmcore != NULL) gk_gkmcoreDel(gkmcore, *ptr1);
  }
  *ptr1 = NULL;

  va_start(plist, ptr1);
  while ((ptr = va_arg(plist, void **)) != LTERM) {
    if (*ptr != NULL) {
      free(*ptr);

      /* remove this memory de-allocation */
      if (gkmcore != NULL) gk_gkmcoreDel(gkmcore, *ptr);
    }
    *ptr = NULL;
  }
  va_end(plist);
}          


/*************************************************************************
* This function returns the current ammount of dynamically allocated
* memory that is used by the system
**************************************************************************/
size_t gk_GetCurMemoryUsed()
{
  if (gkmcore == NULL)
    return 0;
  else
    return gkmcore->cur_hallocs;
}


/*************************************************************************
* This function returns the maximum ammount of dynamically allocated 
* memory that was used by the system
**************************************************************************/
size_t gk_GetMaxMemoryUsed()
{
  if (gkmcore == NULL)
    return 0;
  else
    return gkmcore->max_hallocs;
}
