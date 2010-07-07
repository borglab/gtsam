/* ========================================================================== */
/* === Core/cholmod_error =================================================== */
/* ========================================================================== */

/* -----------------------------------------------------------------------------
 * CHOLMOD/Core Module.  Copyright (C) 2005-2006,
 * Univ. of Florida.  Author: Timothy A. Davis
 * The CHOLMOD/Core Module is licensed under Version 2.1 of the GNU
 * Lesser General Public License.  See lesser.txt for a text of the license.
 * CHOLMOD is also available under other licenses; contact authors for details.
 * http://www.cise.ufl.edu/research/sparse
 * -------------------------------------------------------------------------- */

/* CHOLMOD error-handling routine.  */

#include "cholmod_internal.h"
#include "cholmod_core.h"

/* ========================================================================== */
/* ==== cholmod_error ======================================================= */
/* ========================================================================== */

/* An error has occurred.  Set the status, optionally print an error message,
 * and call the user error-handling routine (if it exists).  If
 * Common->try_catch is TRUE, then CHOLMOD is inside a try/catch block.
 * The status is set, but no message is printed and the user error handler
 * is not called.  This is not (yet) an error, since CHOLMOD may recover.
 *
 * In the current version, this try/catch mechanism is used internally only in
 * cholmod_analyze, which tries multiple ordering methods and picks the best
 * one.  If one or more ordering method fails, it keeps going.  Only one
 * ordering needs to succeed for cholmod_analyze to succeed.
 */

int CHOLMOD(error)
(
    /* ---- input ---- */
    int status,		/* error status */
    const char *file,	/* name of source code file where error occured */ 
    int line,		/* line number in source code file where error occured*/
    const char *message,    /* error message */
    /* --------------- */
    cholmod_common *Common
)
{
    RETURN_IF_NULL_COMMON (FALSE) ;

    Common->status = status ;

    if (!(Common->try_catch))
    {

#ifndef NPRINT
	/* print a warning or error message */
	if (Common->print_function != NULL)
	{
	    if (status > 0 && Common->print > 1)
	    {
		(Common->print_function) ("CHOLMOD warning: %s\n", message) ;
		fflush (stdout) ;
		fflush (stderr) ;
	    }
	    else if (Common->print > 0)
	    {
		(Common->print_function) ("CHOLMOD error: %s\n", message) ;
		fflush (stdout) ;
		fflush (stderr) ;
	    }
	}
#endif

	/* call the user error handler, if it exists */
	if (Common->error_handler != NULL)
	{
	    Common->error_handler (status, file, line, message) ;
	}
    }

    return (TRUE) ;
}
