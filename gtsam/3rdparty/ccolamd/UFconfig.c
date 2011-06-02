/* ========================================================================== */
/* === UFconfig ============================================================= */
/* ========================================================================== */

/* Copyright (c) 2009, University of Florida.  No licensing restrictions
 * apply to this file or to the UFconfig directory.  Author: Timothy A. Davis.
 */

#include "UFconfig.h"

/* -------------------------------------------------------------------------- */
/* UFmalloc: malloc wrapper */
/* -------------------------------------------------------------------------- */

void *UFmalloc              /* pointer to allocated block of memory */
(
    size_t nitems,          /* number of items to malloc (>=1 is enforced) */
    size_t size_of_item,    /* sizeof each item */
    int *ok,                /* TRUE if successful, FALSE otherwise */
    UFconfig *config        /* SuiteSparse-wide configuration */
)
{
    void *p ;
    if (nitems < 1) nitems = 1 ;
    if (nitems * size_of_item != ((double) nitems) * size_of_item)
    {
        /* Int overflow */
        *ok = 0 ;
        return (NULL) ;
    }
    if (!config || config->malloc_memory == NULL)
    {
        /* use malloc by default */
        p = (void *) malloc (nitems * size_of_item) ;
    }
    else
    {
        /* use the pointer to malloc in the config */
        p = (void *) (config->malloc_memory) (nitems * size_of_item) ;
    }
    *ok = (p != NULL) ;
    return (p) ;
}


/* -------------------------------------------------------------------------- */
/* UFfree: free wrapper */
/* -------------------------------------------------------------------------- */

void *UFfree                /* always returns NULL */
(
    void *p,                /* block to free */
    UFconfig *config        /* SuiteSparse-wide configuration */
)
{
    if (p)
    {
        if (!config || config->free_memory == NULL)
        {
            /* use free by default */
            free (p) ;
        }
        else
        {
            /* use the pointer to free in the config */
            (config->free_memory) (p) ;
        }
    }
    return (NULL) ;
}

