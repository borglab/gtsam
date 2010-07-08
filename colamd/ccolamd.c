/* ========================================================================== */
/* === CCOLAMD/CSYMAMD - a constrained column ordering algorithm ============ */
/* ========================================================================== */

/* ----------------------------------------------------------------------------
 * CCOLAMD, Copyright (C) Univ. of Florida.  Authors: Timothy A. Davis,
 * Sivasankaran Rajamanickam, and Stefan Larimore
 * See License.txt for the Version 2.1 of the GNU Lesser General Public License
 * http://www.cise.ufl.edu/research/sparse
 * -------------------------------------------------------------------------- */

/*
 *  ccolamd:  a constrained approximate minimum degree column ordering
 *	algorithm, LU factorization of symmetric or unsymmetric matrices,
 *	QR factorization, least squares, interior point methods for
 *	linear programming problems, and other related problems.
 *
 *  csymamd:  a constrained approximate minimum degree ordering algorithm for
 *	Cholesky factorization of symmetric matrices.
 *
 *  Purpose:
 *
 *	CCOLAMD computes a permutation Q such that the Cholesky factorization of
 *	(AQ)'(AQ) has less fill-in and requires fewer floating point operations
 *	than A'A.  This also provides a good ordering for sparse partial
 *	pivoting methods, P(AQ) = LU, where Q is computed prior to numerical
 *	factorization, and P is computed during numerical factorization via
 *	conventional partial pivoting with row interchanges.  CCOLAMD is an
 *	extension of COLAMD, available as built-in function in MATLAB Version 6,
 *	available from MathWorks, Inc. (http://www.mathworks.com).  This
 *	routine can be used in place of COLAMD in MATLAB.
 *
 *	CSYMAMD computes a permutation P of a symmetric matrix A such that the
 *	Cholesky factorization of PAP' has less fill-in and requires fewer
 *	floating point operations than A.  CSYMAMD constructs a matrix M such
 *	that M'M has the same nonzero pattern of A, and then orders the columns
 *	of M using colmmd.  The column ordering of M is then returned as the
 *	row and column ordering P of A.  CSYMAMD is an extension of SYMAMD.
 *
 *  Authors:
 *
 *	Timothy A. Davis and S. Rajamanickam wrote CCOLAMD, based directly on
 *	COLAMD by Stefan I. Larimore and Timothy A. Davis, University of
 *	Florida.  The algorithm was developed in collaboration with John
 *	Gilbert, (UCSB, then at Xerox PARC), and Esmond Ng, (Lawrence Berkeley
 *	National Lab, then at Oak Ridge National Laboratory).
 *
 *  Acknowledgements:
 *
 *	This work was supported by the National Science Foundation, under
 *	grants DMS-9504974 and DMS-9803599, CCR-0203270, and a grant from the
 *	Sandia National Laboratory (Dept. of Energy).
 *
 *  Copyright and License:
 *
 *	Copyright (c) 1998-2005 by the University of Florida.
 *	All Rights Reserved.
 *	COLAMD is also available under alternate licenses, contact T. Davis
 *	for details.
 *
 *	This library is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	This library is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *	Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with this library; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 *	USA
 *
 *	Permission is hereby granted to use or copy this program under the
 *	terms of the GNU LGPL, provided that the Copyright, this License,
 *	and the Availability of the original version is retained on all copies.
 *	User documentation of any code that uses this code or any modified
 *	version of this code must cite the Copyright, this License, the
 *	Availability note, and "Used by permission." Permission to modify
 *	the code and to distribute modified code is granted, provided the
 *	Copyright, this License, and the Availability note are retained,
 *	and a notice that the code was modified is included.
 *
 *  Availability:
 *
 *	The CCOLAMD/CSYMAMD library is available at
 *
 *	    http://www.cise.ufl.edu/research/sparse/ccolamd/
 *
 *	This is the http://www.cise.ufl.edu/research/sparse/ccolamd/ccolamd.c
 *	file.
 *
 *   See the ChangeLog file for changes since Version 1.0.
 */

/* ========================================================================== */
/* === Description of user-callable routines ================================ */
/* ========================================================================== */

/* CCOLAMD includes both int and UF_long versions of all its routines.  The
 * description below is for the int version.   For UF_long, all int arguments
 * become UF_long integers.  UF_long is normally defined as long, except for
 * WIN64 */

/*  ----------------------------------------------------------------------------
 *  ccolamd_recommended:
 *  ----------------------------------------------------------------------------
 *
 *	C syntax:
 *
 *	    #include "ccolamd.h"
 *	    size_t ccolamd_recommended (int nnz, int n_row, int n_col) ;
 *	    size_t ccolamd_l_recommended (UF_long nnz, UF_long n_row,
 *		UF_long n_col) ;
 *
 *	Purpose:
 *
 *	    Returns recommended value of Alen for use by ccolamd.  Returns 0
 *	    if any input argument is negative.  The use of this routine
 *	    is optional.  Not needed for csymamd, which dynamically allocates
 *	    its own memory.
 *
 *	Arguments (all input arguments):
 *
 *	    int nnz ;		Number of nonzeros in the matrix A.  This must
 *				be the same value as p [n_col] in the call to
 *				ccolamd - otherwise you will get a wrong value
 *				of the recommended memory to use.
 *
 *	    int n_row ;		Number of rows in the matrix A.
 *
 *	    int n_col ;		Number of columns in the matrix A.
 *
 *  ----------------------------------------------------------------------------
 *  ccolamd_set_defaults:
 *  ----------------------------------------------------------------------------
 *
 *	C syntax:
 *
 *	    #include "ccolamd.h"
 *	    ccolamd_set_defaults (double knobs [CCOLAMD_KNOBS]) ;
 *	    ccolamd_l_set_defaults (double knobs [CCOLAMD_KNOBS]) ;
 *
 *	Purpose:
 *
 *	    Sets the default parameters.  The use of this routine is optional.
 *	    Passing a (double *) NULL pointer for the knobs results in the
 *	    default parameter settings.
 *
 *	Arguments:
 *
 *	    double knobs [CCOLAMD_KNOBS] ;	Output only.
 *
 *	    knobs [0] and knobs [1] behave differently than they did in COLAMD.
 *	    The other knobs are new to CCOLAMD.
 *
 *	    knobs [0]: dense row control
 *
 *		For CCOLAMD, rows with more than
 *		max (16, knobs [CCOLAMD_DENSE_ROW] * sqrt (n_col))
 *		entries are removed prior to ordering.
 *
 *		For CSYMAMD, rows and columns with more than
 *		max (16, knobs [CCOLAMD_DENSE_ROW] * sqrt (n))
 *		entries are removed prior to ordering, and placed last in the
 *		output ordering (subject to the constraints).
 *
 *		If negative, only completely dense rows are removed.  If you
 *		intend to use CCOLAMD for a Cholesky factorization of A*A', set
 *		knobs [CCOLAMD_DENSE_ROW] to -1, which is more appropriate for
 *		that case.
 *
 *		Default: 10.
 *
 *	    knobs [1]: dense column control
 *
 *		For CCOLAMD, columns with more than
 *		max (16, knobs [CCOLAMD_DENSE_COL] * sqrt (MIN (n_row,n_col)))
 *		entries are removed prior to ordering, and placed last in the
 *		output column ordering (subject to the constraints).
 *		Not used by CSYMAMD.  If negative, only completely dense
 *		columns are removed.  Default: 10.
 *
 *	    knobs [2]: aggressive absorption
 *
 *	        knobs [CCOLAMD_AGGRESSIVE] controls whether or not to do
 *	        aggressive absorption during the ordering.  Default is TRUE
 *	        (nonzero).  If zero, no aggressive absorption is performed.
 *
 *	    knobs [3]: optimize ordering for LU or Cholesky
 *
 *		knobs [CCOLAMD_LU] controls an option that optimizes the
 *		ordering for the LU of A or the Cholesky factorization of A'A.
 *		If TRUE (nonzero), an ordering optimized for LU is performed.
 *		If FALSE (zero), an ordering for Cholesky is performed.
 *		Default is FALSE.  CSYMAMD ignores this parameter; it always
 *		orders for Cholesky.
 *
 *  ----------------------------------------------------------------------------
 *  ccolamd:
 *  ----------------------------------------------------------------------------
 *
 *	C syntax:
 *
 *	    #include "ccolamd.h"
 *	    int ccolamd (int n_row, int n_col, int Alen, int *A, int *p,
 *	    	double knobs [CCOLAMD_KNOBS], int stats [CCOLAMD_STATS],
 *		int *cmember) ;
 *
 *	    UF_long ccolamd_l (UF_long n_row, UF_long n_col, UF_long Alen,
 *		UF_long *A, UF_long *p, double knobs [CCOLAMD_KNOBS],
 *		UF_long stats [CCOLAMD_STATS], UF_long *cmember) ;
 *
 *	Purpose:
 *
 *	    Computes a column ordering (Q) of A such that P(AQ)=LU or
 *	    (AQ)'AQ=LL' have less fill-in and require fewer floating point
 *	    operations than factorizing the unpermuted matrix A or A'A,
 *	    respectively.
 *
 *	Returns:
 *
 *	    TRUE (1) if successful, FALSE (0) otherwise.
 *
 *	Arguments (for int version):
 *
 *	    int n_row ;		Input argument.
 *
 *		Number of rows in the matrix A.
 *		Restriction:  n_row >= 0.
 *		ccolamd returns FALSE if n_row is negative.
 *
 *	    int n_col ;		Input argument.
 *
 *		Number of columns in the matrix A.
 *		Restriction:  n_col >= 0.
 *		ccolamd returns FALSE if n_col is negative.
 *
 *	    int Alen ;		Input argument.
 *
 *		Restriction (see note):
 *		Alen >= MAX (2*nnz, 4*n_col) + 17*n_col + 7*n_row + 7, where
 *		nnz = p [n_col].  ccolamd returns FALSE if this condition is
 *		not met. We recommend about nnz/5 more space for better
 *		efficiency.  This restriction makes an modest assumption
 *		regarding the size of two typedef'd structures in ccolamd.h.
 *		We do, however, guarantee that
 *
 *		    Alen >= ccolamd_recommended (nnz, n_row, n_col)
 *
 *		will work efficiently.
 *
 *	    int A [Alen] ;	Input argument, undefined on output.
 *
 *		A is an integer array of size Alen.  Alen must be at least as
 *		large as the bare minimum value given above, but this is very
 *		low, and can result in excessive run time.  For best
 *		performance, we recommend that Alen be greater than or equal to
 *		ccolamd_recommended (nnz, n_row, n_col), which adds
 *		nnz/5 to the bare minimum value given above.
 *
 *		On input, the row indices of the entries in column c of the
 *		matrix are held in A [(p [c]) ... (p [c+1]-1)].  The row indices
 *		in a given column c need not be in ascending order, and
 *		duplicate row indices may be be present.  However, ccolamd will
 *		work a little faster if both of these conditions are met
 *		(ccolamd puts the matrix into this format, if it finds that the
 *		the conditions are not met).
 *
 *		The matrix is 0-based.  That is, rows are in the range 0 to
 *		n_row-1, and columns are in the range 0 to n_col-1.  ccolamd
 *		returns FALSE if any row index is out of range.
 *
 *		The contents of A are modified during ordering, and are
 *		undefined on output.
 *
 *	    int p [n_col+1] ;	Both input and output argument.
 *
 *		p is an integer array of size n_col+1.  On input, it holds the
 *		"pointers" for the column form of the matrix A.  Column c of
 *		the matrix A is held in A [(p [c]) ... (p [c+1]-1)].  The first
 *		entry, p [0], must be zero, and p [c] <= p [c+1] must hold
 *		for all c in the range 0 to n_col-1.  The value nnz = p [n_col]
 *		is thus the total number of entries in the pattern of the
 *		matrix A.  ccolamd returns FALSE if these conditions are not
 *		met.
 *
 *		On output, if ccolamd returns TRUE, the array p holds the column
 *		permutation (Q, for P(AQ)=LU or (AQ)'(AQ)=LL'), where p [0] is
 *		the first column index in the new ordering, and p [n_col-1] is
 *		the last.  That is, p [k] = j means that column j of A is the
 *		kth pivot column, in AQ, where k is in the range 0 to n_col-1
 *		(p [0] = j means that column j of A is the first column in AQ).
 *
 *		If ccolamd returns FALSE, then no permutation is returned, and
 *		p is undefined on output.
 *
 *	    double knobs [CCOLAMD_KNOBS] ;	Input argument.
 *
 *		See ccolamd_set_defaults for a description.
 *
 *	    int stats [CCOLAMD_STATS] ;		Output argument.
 *
 *		Statistics on the ordering, and error status.
 *		See ccolamd.h for related definitions.
 *		ccolamd returns FALSE if stats is not present.
 *
 *		stats [0]:  number of dense or empty rows ignored.
 *
 *		stats [1]:  number of dense or empty columns ignored (and
 *		    ordered last in the output permutation p, subject to the
 *		    constraints).  Note that a row can become "empty" if it
 *		    contains only "dense" and/or "empty" columns, and similarly
 *		    a column can become "empty" if it only contains "dense"
 *		    and/or "empty" rows.
 *
 *		stats [2]:  number of garbage collections performed.  This can
 *		    be excessively high if Alen is close to the minimum
 *		    required value.
 *
 *		stats [3]:  status code.  < 0 is an error code.
 *			    > 1 is a warning or notice.
 *
 *		    0	OK.  Each column of the input matrix contained row
 *			indices in increasing order, with no duplicates.
 *
 *		    1	OK, but columns of input matrix were jumbled (unsorted
 *			columns or duplicate entries).  CCOLAMD had to do some
 *			extra work to sort the matrix first and remove
 *			duplicate entries, but it still was able to return a
 *			valid permutation (return value of ccolamd was TRUE).
 *
 *			stats [4]: highest column index of jumbled columns
 *			stats [5]: last seen duplicate or unsorted row index
 *			stats [6]: number of duplicate or unsorted row indices
 *
 *		    -1	A is a null pointer
 *
 *		    -2	p is a null pointer
 *
 *		    -3 	n_row is negative.  stats [4]: n_row
 *
 *		    -4	n_col is negative.  stats [4]: n_col
 *
 *		    -5	number of nonzeros in matrix is negative
 *
 *			stats [4]: number of nonzeros, p [n_col]
 *
 *		    -6	p [0] is nonzero
 *
 *			stats [4]: p [0]
 *
 *		    -7	A is too small
 *
 *			stats [4]: required size
 *			stats [5]: actual size (Alen)
 *
 *		    -8	a column has a negative number of entries
 *
 *			stats [4]: column with < 0 entries
 *			stats [5]: number of entries in col
 *
 *		    -9	a row index is out of bounds
 *
 *			stats [4]: column with bad row index
 *			stats [5]: bad row index
 *			stats [6]: n_row, # of rows of matrx
 *
 *		    -10	(unused; see csymamd)
 *
 *	    int cmember [n_col] ;		Input argument.
 *
 *		cmember is new to CCOLAMD.  It did not appear in COLAMD.
 *		It places contraints on the output ordering.  s = cmember [j]
 *		gives the constraint set s that contains the column j
 *		(Restriction: 0 <= s < n_col).  In the output column
 *		permutation, all columns in set 0 appear first, followed by
 *		all columns in set 1, and so on.  If NULL, all columns are
 *		treated as if they were in a single constraint set, and you
 *		will obtain the same ordering as COLAMD (with one exception:
 *		the dense row/column threshold and other default knobs in
 *		CCOLAMD and COLAMD are different).
 *
 *	Example:
 *
 *	    See
 *	    http://www.cise.ufl.edu/research/sparse/ccolamd/ccolamd_example.c
 *	    for a complete example.
 *
 *	    To order the columns of a 5-by-4 matrix with 11 nonzero entries in
 *	    the following nonzero pattern
 *
 *	    	x 0 x 0
 *		x 0 x x
 *		0 x x 0
 *		0 0 x x
 *		x x 0 0
 *
 *	    with default knobs, no output statistics, and no ordering
 *	    constraints, do the following:
 *
 *		#include "ccolamd.h"
 *		#define ALEN 144
 *		int A [ALEN] = {0, 1, 4, 2, 4, 0, 1, 2, 3, 1, 3} ;
 *		int p [ ] = {0, 3, 5, 9, 11} ;
 *		int stats [CCOLAMD_STATS] ;
 *		ccolamd (5, 4, ALEN, A, p, (double *) NULL, stats, NULL) ;
 *
 *	    The permutation is returned in the array p, and A is destroyed.
 *
 *  ----------------------------------------------------------------------------
 *  csymamd:
 *  ----------------------------------------------------------------------------
 *
 *	C syntax:
 *
 *	    #include "ccolamd.h"
 *
 *	    int csymamd (int n, int *A, int *p, int *perm,
 *	    	double knobs [CCOLAMD_KNOBS], int stats [CCOLAMD_STATS],
 *		void (*allocate) (size_t, size_t), void (*release) (void *),
 *		int *cmember, int stype) ;
 *
 *	    UF_long csymamd_l (UF_long n, UF_long *A, UF_long *p, UF_long *perm,
 *	    	double knobs [CCOLAMD_KNOBS], UF_long stats [CCOLAMD_STATS],
 *		void (*allocate) (size_t, size_t), void (*release) (void *),
 *		UF_long *cmember, UF_long stype) ;
 *
 *	Purpose:
 *
 *  	    The csymamd routine computes an ordering P of a symmetric sparse
 *	    matrix A such that the Cholesky factorization PAP' = LL' remains
 *	    sparse.  It is based on a column ordering of a matrix M constructed
 *	    so that the nonzero pattern of M'M is the same as A.  Either the
 *	    lower or upper triangular part of A can be used, or the pattern
 *	    A+A' can be used.  You must pass your selected memory allocator
 *	    (usually calloc/free or mxCalloc/mxFree) to csymamd, for it to
 *	    allocate memory for the temporary matrix M.
 *
 *	Returns:
 *
 *	    TRUE (1) if successful, FALSE (0) otherwise.
 *
 *	Arguments:
 *
 *	    int n ;		Input argument.
 *
 *	    	Number of rows and columns in the symmetrix matrix A.
 *		Restriction:  n >= 0.
 *		csymamd returns FALSE if n is negative.
 *
 *	    int A [nnz] ;	Input argument.
 *
 *	    	A is an integer array of size nnz, where nnz = p [n].
 *
 *		The row indices of the entries in column c of the matrix are
 *		held in A [(p [c]) ... (p [c+1]-1)].  The row indices in a
 *		given column c need not be in ascending order, and duplicate
 *		row indices may be present.  However, csymamd will run faster
 *		if the columns are in sorted order with no duplicate entries.
 *
 *		The matrix is 0-based.  That is, rows are in the range 0 to
 *		n-1, and columns are in the range 0 to n-1.  csymamd
 *		returns FALSE if any row index is out of range.
 *
 *		The contents of A are not modified.
 *
 *	    int p [n+1] ;   	Input argument.
 *
 *		p is an integer array of size n+1.  On input, it holds the
 *		"pointers" for the column form of the matrix A.  Column c of
 *		the matrix A is held in A [(p [c]) ... (p [c+1]-1)].  The first
 *		entry, p [0], must be zero, and p [c] <= p [c+1] must hold
 *		for all c in the range 0 to n-1.  The value p [n] is
 *		thus the total number of entries in the pattern of the matrix A.
 *		csymamd returns FALSE if these conditions are not met.
 *
 *		The contents of p are not modified.
 *
 *	    int perm [n+1] ;   	Output argument.
 *
 *		On output, if csymamd returns TRUE, the array perm holds the
 *		permutation P, where perm [0] is the first index in the new
 *		ordering, and perm [n-1] is the last.  That is, perm [k] = j
 *		means that row and column j of A is the kth column in PAP',
 *		where k is in the range 0 to n-1 (perm [0] = j means
 *		that row and column j of A are the first row and column in
 *		PAP').  The array is used as a workspace during the ordering,
 *		which is why it must be of length n+1, not just n.
 *
 *	    double knobs [CCOLAMD_KNOBS] ;	Input argument.
 *
 *		See colamd_set_defaults for a description.
 *
 *	    int stats [CCOLAMD_STATS] ;		Output argument.
 *
 *		Statistics on the ordering, and error status.
 *		See ccolamd.h for related definitions.
 *		csymand returns FALSE if stats is not present.
 *
 *		stats [0]:  number of dense or empty row and columns ignored
 *		    (and ordered last in the output permutation perm, subject
 *		    to the constraints).  Note that a row/column can become
 *		    "empty" if it contains only "dense" and/or "empty"
 *		    columns/rows.
 *
 *		stats [1]:  (same as stats [0])
 *
 *		stats [2]:  number of garbage collections performed.
 *
 *		stats [3]:  status code.  < 0 is an error code.
 *			    > 1 is a warning or notice.
 *
 *		    0 to -9: same as ccolamd, with n replacing n_col and n_row,
 *			and -3 and -7 are unused.
 *
 *		    -10	out of memory (unable to allocate temporary workspace
 *			    for M or count arrays using the "allocate" routine
 *			    passed into csymamd).
 *
 *	    void * (*allocate) (size_t, size_t)
 *
 *	    	A pointer to a function providing memory allocation.  The
 *		allocated memory must be returned initialized to zero.  For a
 *		C application, this argument should normally be a pointer to
 *		calloc.  For a MATLAB mexFunction, the routine mxCalloc is
 *		passed instead.
 *
 *	    void (*release) (size_t, size_t)
 *
 *	    	A pointer to a function that frees memory allocated by the
 *		memory allocation routine above.  For a C application, this
 *		argument should normally be a pointer to free.  For a MATLAB
 *		mexFunction, the routine mxFree is passed instead.
 *
 *	    int cmember [n] ;		Input argument.
 *
 *		Same as ccolamd, except that cmember is of size n, and it places
 *		contraints symmetrically, on both the row and column ordering.
 *		Entries in cmember must be in the range 0 to n-1.
 *
 *	    int stype ;			Input argument.
 *
 *		If stype < 0, then only the strictly lower triangular part of
 *		A is accessed.  The upper triangular part is assumed to be the
 *		transpose of the lower triangular part.  This is the same as
 *		SYMAMD, which did not have an stype parameter.
 *
 *		If stype > 0, only the strictly upper triangular part of A is
 *		accessed.  The lower triangular part is assumed to be the
 *		transpose of the upper triangular part.
 *
 *		If stype == 0, then the nonzero pattern of A+A' is ordered.
 *
 *  ----------------------------------------------------------------------------
 *  ccolamd_report:
 *  ----------------------------------------------------------------------------
 *
 *	C syntax:
 *
 *	    #include "ccolamd.h"
 *	    ccolamd_report (int stats [CCOLAMD_STATS]) ;
 *	    ccolamd_l_report (UF_long stats [CCOLAMD_STATS]) ;
 *
 *	Purpose:
 *
 *	    Prints the error status and statistics recorded in the stats
 *	    array on the standard error output (for a standard C routine)
 *	    or on the MATLAB output (for a mexFunction).
 *
 *	Arguments:
 *
 *	    int stats [CCOLAMD_STATS] ;	Input only.  Statistics from ccolamd.
 *
 *
 *  ----------------------------------------------------------------------------
 *  csymamd_report:
 *  ----------------------------------------------------------------------------
 *
 *	C syntax:
 *
 *	    #include "ccolamd.h"
 *	    csymamd_report (int stats [CCOLAMD_STATS]) ;
 *	    csymamd_l_report (UF_long stats [CCOLAMD_STATS]) ;
 *
 *	Purpose:
 *
 *	    Prints the error status and statistics recorded in the stats
 *	    array on the standard error output (for a standard C routine)
 *	    or on the MATLAB output (for a mexFunction).
 *
 *	Arguments:
 *
 *	    int stats [CCOLAMD_STATS] ;	Input only.  Statistics from csymamd.
 *
 */


/* ========================================================================== */
/* === Scaffolding code definitions  ======================================== */
/* ========================================================================== */

/* Ensure that debugging is turned off: */
#ifndef NDEBUG
#define NDEBUG
#endif

/* turn on debugging by uncommenting the following line
 #undef NDEBUG
 */

/* ========================================================================== */
/* === Include files ======================================================== */
/* ========================================================================== */

#include "ccolamd.h"

#include <stdlib.h>
#include <math.h>
#include <limits.h>

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#include "matrix.h"
#endif

#if !defined (NPRINT) || !defined (NDEBUG)
#include <stdio.h>
#endif

#ifndef NULL
#define NULL ((void *) 0)
#endif

/* ========================================================================== */
/* === int or UF_long ======================================================= */
/* ========================================================================== */

/* define UF_long */
#include "UFconfig.h"

#ifdef DLONG

#define Int UF_long
#define ID  UF_long_id
#define Int_MAX UF_long_max

#define CCOLAMD_recommended ccolamd_l_recommended
#define CCOLAMD_set_defaults ccolamd_l_set_defaults
#define CCOLAMD_2 ccolamd2_l
#define CCOLAMD_MAIN ccolamd_l
#define CCOLAMD_apply_order ccolamd_l_apply_order
#define CCOLAMD_postorder ccolamd_l_postorder
#define CCOLAMD_post_tree ccolamd_l_post_tree
#define CCOLAMD_fsize ccolamd_l_fsize
#define CSYMAMD_MAIN csymamd_l
#define CCOLAMD_report ccolamd_l_report
#define CSYMAMD_report csymamd_l_report

#else

#define Int int
#define ID "%d"
#define Int_MAX INT_MAX

#define CCOLAMD_recommended ccolamd_recommended
#define CCOLAMD_set_defaults ccolamd_set_defaults
#define CCOLAMD_2 ccolamd2
#define CCOLAMD_MAIN ccolamd
#define CCOLAMD_apply_order ccolamd_apply_order
#define CCOLAMD_postorder ccolamd_postorder
#define CCOLAMD_post_tree ccolamd_post_tree
#define CCOLAMD_fsize ccolamd_fsize
#define CSYMAMD_MAIN csymamd
#define CCOLAMD_report ccolamd_report
#define CSYMAMD_report csymamd_report

#endif

/* ========================================================================== */
/* === Row and Column structures ============================================ */
/* ========================================================================== */

typedef struct CColamd_Col_struct
{
    /* size of this struct is 8 integers if no padding occurs */

    Int start ;		/* index for A of first row in this column, or DEAD */
			/* if column is dead */
    Int length ;	/* number of rows in this column */
    union
    {
	Int thickness ;	/* number of original columns represented by this */
			/* col, if the column is alive */
	Int parent ;	/* parent in parent tree super-column structure, if */
			/* the column is dead */
    } shared1 ;
    union
    {
	Int score ;	
	Int order ;
    } shared2 ; 
    union
    {
	Int headhash ;	/* head of a hash bucket, if col is at the head of */
			/* a degree list */
	Int hash ;	/* hash value, if col is not in a degree list */
	Int prev ;	/* previous column in degree list, if col is in a */
			/* degree list (but not at the head of a degree list) */
    } shared3 ;
    union
    {
	Int degree_next ;	/* next column, if col is in a degree list */
	Int hash_next ;		/* next column, if col is in a hash list */
    } shared4 ;

    Int nextcol ;       /* next column in this supercolumn */
    Int lastcol ;       /* last column in this supercolumn */

} CColamd_Col ;


typedef struct CColamd_Row_struct
{
    /* size of this struct is 6 integers if no padding occurs */

    Int start ;		/* index for A of first col in this row */
    Int length ;	/* number of principal columns in this row */
    union
    {
	Int degree ;	/* number of principal & non-principal columns in row */
	Int p ;		/* used as a row pointer in init_rows_cols () */
    } shared1 ;
    union
    {
	Int mark ;	/* for computing set differences and marking dead rows*/
	Int first_column ;/* first column in row (used in garbage collection) */
    } shared2 ;

    Int thickness ;     /* number of original rows represented by this row */
                        /* that are not yet pivotal */
    Int front ;         /* -1 if an original row */
    			/* k if this row represents the kth frontal matrix */
                        /* where k goes from 0 to at most n_col-1 */

} CColamd_Row ;

/* ========================================================================== */
/* === basic definitions ==================================================== */
/* ========================================================================== */

#define EMPTY (-1)
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

/* Routines are either PUBLIC (user-callable) or PRIVATE (not user-callable) */
#define GLOBAL 
#define PUBLIC
#define PRIVATE static 

#define DENSE_DEGREE(alpha,n) \
    ((Int) MAX (16.0, (alpha) * sqrt ((double) (n))))

#define CMEMBER(c) ((cmember == (Int *) NULL) ? (0) : (cmember [c]))

/* True if x is NaN */
#define SCALAR_IS_NAN(x)        ((x) != (x))

/* true if an integer (stored in double x) would overflow (or if x is NaN) */
#define INT_OVERFLOW(x) ((!((x) * (1.0+1e-8) <= (double) Int_MAX)) \
                        || SCALAR_IS_NAN (x))

#define ONES_COMPLEMENT(r) (-(r)-1)
#undef TRUE
#undef FALSE
#define TRUE (1)
#define FALSE (0)

/* Row and column status */
#define ALIVE	(0)
#define DEAD	(-1)

/* Column status */
#define DEAD_PRINCIPAL		(-1)
#define DEAD_NON_PRINCIPAL	(-2)

/* Macros for row and column status update and checking. */
#define ROW_IS_DEAD(r)			ROW_IS_MARKED_DEAD (Row[r].shared2.mark)
#define ROW_IS_MARKED_DEAD(row_mark)	(row_mark < ALIVE)
#define ROW_IS_ALIVE(r)			(Row [r].shared2.mark >= ALIVE)
#define COL_IS_DEAD(c)			(Col [c].start < ALIVE)
#define COL_IS_ALIVE(c)			(Col [c].start >= ALIVE)
#define COL_IS_DEAD_PRINCIPAL(c)	(Col [c].start == DEAD_PRINCIPAL)
#define KILL_ROW(r)			{ Row [r].shared2.mark = DEAD ; }
#define KILL_PRINCIPAL_COL(c)		{ Col [c].start = DEAD_PRINCIPAL ; }
#define KILL_NON_PRINCIPAL_COL(c)	{ Col [c].start = DEAD_NON_PRINCIPAL ; }


/* ========================================================================== */
/* === ccolamd reporting mechanism ========================================== */
/* ========================================================================== */

#if defined (MATLAB_MEX_FILE) || defined (MATHWORKS)
/* In MATLAB, matrices are 1-based to the user, but 0-based internally */
#define INDEX(i) ((i)+1)
#else
/* In C, matrices are 0-based and indices are reported as such in *_report */
#define INDEX(i) (i)
#endif

/* All output goes through the PRINTF macro.  */
#define PRINTF(params) { if (ccolamd_printf != NULL) (void) ccolamd_printf params ; }


/* ========================================================================== */
/* === Debugging prototypes and definitions ================================= */
/* ========================================================================== */

#ifndef NDEBUG

#include <assert.h>

/* debug print level, present only when debugging */
PRIVATE Int ccolamd_debug ;

/* debug print statements */
#define DEBUG0(params) { PRINTF (params) ; }
#define DEBUG1(params) { if (ccolamd_debug >= 1) PRINTF (params) ; }
#define DEBUG2(params) { if (ccolamd_debug >= 2) PRINTF (params) ; }
#define DEBUG3(params) { if (ccolamd_debug >= 3) PRINTF (params) ; }
#define DEBUG4(params) { if (ccolamd_debug >= 4) PRINTF (params) ; }

#ifdef MATLAB_MEX_FILE
#define ASSERT(expression) (mxAssert ((expression), ""))
#else
#define ASSERT(expression) (assert (expression))
#endif

PRIVATE void ccolamd_get_debug
(   
    char *method
) ; 

PRIVATE void debug_mark
(
    Int n_row,
    CColamd_Row Row [],
    Int tag_mark,
    Int max_mark
) ;

PRIVATE void debug_matrix
(
    Int n_row,
    Int n_col,
    CColamd_Row Row [],
    CColamd_Col Col [],
    Int A []
) ;

PRIVATE void debug_structures
(
    Int n_row,
    Int n_col,
    CColamd_Row Row [],
    CColamd_Col Col [],
    Int A [],
    Int in_cset [],
    Int cset_start []
) ;

PRIVATE void dump_super
(
    Int super_c,
    CColamd_Col Col [],
    Int n_col
) ;

PRIVATE void debug_deg_lists
(
    Int n_row,
    Int n_col,
    CColamd_Row Row [ ],
    CColamd_Col Col [ ],
    Int head [ ],
    Int min_score,
    Int should,
    Int max_deg
) ;

#else

/* === No debugging ========================================================= */

#define DEBUG0(params) ;
#define DEBUG1(params) ;
#define DEBUG2(params) ;
#define DEBUG3(params) ;
#define DEBUG4(params) ;

#define ASSERT(expression)

#endif

/* ========================================================================== */
/* === Prototypes of PRIVATE routines ======================================= */
/* ========================================================================== */

PRIVATE Int init_rows_cols
(
    Int n_row,
    Int n_col,
    CColamd_Row Row [ ],
    CColamd_Col Col [ ],
    Int A [ ],
    Int p [ ],
    Int stats [CCOLAMD_STATS]
) ;

PRIVATE void init_scoring
(
    Int n_row,
    Int n_col,
    CColamd_Row Row [ ],
    CColamd_Col Col [ ],
    Int A [ ],
    Int head [ ],
    double knobs [CCOLAMD_KNOBS],
    Int *p_n_row2,
    Int *p_n_col2,
    Int *p_max_deg,
    Int cmember [ ],
    Int n_cset,
    Int cset_start [ ],
    Int dead_cols [ ],
    Int *p_ndense_row,		/* number of dense rows */
    Int *p_nempty_row,		/* number of original empty rows */
    Int *p_nnewlyempty_row,	/* number of newly empty rows */
    Int *p_ndense_col,		/* number of dense cols (excl "empty" cols) */
    Int *p_nempty_col,		/* number of original empty cols */
    Int *p_nnewlyempty_col	/* number of newly empty cols */
) ;

PRIVATE Int find_ordering
(
    Int n_row,
    Int n_col,
    Int Alen,
    CColamd_Row Row [ ],
    CColamd_Col Col [ ],
    Int A [ ],
    Int head [ ],
#ifndef NDEBUG
    Int n_col2,
#endif
    Int max_deg,
    Int pfree,
    Int cset [ ],
    Int cset_start [ ],
#ifndef NDEBUG
    Int n_cset,
#endif
    Int cmember [ ],
    Int Front_npivcol [ ],
    Int Front_nrows [ ],
    Int Front_ncols [ ],
    Int Front_parent [ ],
    Int Front_cols [ ],
    Int *p_nfr,
    Int aggressive,
    Int InFront [ ],
    Int order_for_lu
) ;

PRIVATE void detect_super_cols
(
#ifndef NDEBUG
    Int n_col,
    CColamd_Row Row [ ],
#endif
    CColamd_Col Col [ ],
    Int A [ ],
    Int head [ ],
    Int row_start,
    Int row_length,
    Int in_set [ ]
) ;

PRIVATE Int garbage_collection
(
    Int n_row,
    Int n_col,
    CColamd_Row Row [ ],
    CColamd_Col Col [ ],
    Int A [ ],
    Int *pfree
) ;

PRIVATE Int clear_mark
(
    Int tag_mark,
    Int max_mark,
    Int n_row,
    CColamd_Row Row [ ]
) ;

PRIVATE void print_report
(
    char *method,
    Int stats [CCOLAMD_STATS]
) ;


/* ========================================================================== */
/* === USER-CALLABLE ROUTINES: ============================================== */
/* ========================================================================== */


/* ========================================================================== */
/* === ccolamd_recommended ================================================== */
/* ========================================================================== */

/*
 *  The ccolamd_recommended routine returns the suggested size for Alen.  This
 *  value has been determined to provide good balance between the number of
 *  garbage collections and the memory requirements for ccolamd.  If any
 *  argument is negative, or if integer overflow occurs, a 0 is returned as
 *  an error condition.
 *
 *  2*nnz space is required for the row and column indices of the matrix
 *  (or 4*n_col, which ever is larger).
 *
 *  CCOLAMD_C (n_col) + CCOLAMD_R (n_row) space is required for the Col and Row
 *  arrays, respectively, which are internal to ccolamd.  This is equal to
 *  8*n_col + 6*n_row if the structures are not padded.
 *
 *  An additional n_col space is the minimal amount of "elbow room",
 *  and nnz/5 more space is recommended for run time efficiency.
 *
 *  The remaining (((3 * n_col) + 1) + 5 * (n_col + 1) + n_row) space is
 *  for other workspace used in ccolamd which did not appear in colamd.
 */

/* add two values of type size_t, and check for integer overflow */
static size_t t_add (size_t a, size_t b, int *ok)
{
    (*ok) = (*ok) && ((a + b) >= MAX (a,b)) ;
    return ((*ok) ? (a + b) : 0) ;
}

/* compute a*k where k is a small integer, and check for integer overflow */
static size_t t_mult (size_t a, size_t k, int *ok)
{
    size_t i, s = 0 ;
    for (i = 0 ; i < k ; i++)
    {
	s = t_add (s, a, ok) ;
    }
    return (s) ;
}

/* size of the Col and Row structures */
#define CCOLAMD_C(n_col,ok) \
    ((t_mult (t_add (n_col, 1, ok), sizeof (CColamd_Col), ok) / sizeof (Int)))

#define CCOLAMD_R(n_row,ok) \
    ((t_mult (t_add (n_row, 1, ok), sizeof (CColamd_Row), ok) / sizeof (Int)))

/*
#define CCOLAMD_RECOMMENDED(nnz, n_row, n_col) \
	    MAX (2 * nnz, 4 * n_col) + \
	    CCOLAMD_C (n_col) + CCOLAMD_R (n_row) + n_col + (nnz / 5) \
	    + ((3 * n_col) + 1) + 5 * (n_col + 1) + n_row
 */

static size_t ccolamd_need (Int nnz, Int n_row, Int n_col, int *ok)
{

    /* ccolamd_need, compute the following, and check for integer overflow:
	need = MAX (2*nnz, 4*n_col) + n_col +
		Col_size + Row_size +
		(3*n_col+1) + (5*(n_col+1)) + n_row ;
    */
    size_t s, c, r, t ;

    /* MAX (2*nnz, 4*n_col) */
    s = t_mult (nnz, 2, ok) ;	    /* 2*nnz */
    t = t_mult (n_col, 4, ok) ;	    /* 4*n_col */
    s = MAX (s,t) ;

    s = t_add (s, n_col, ok) ;	    /* bare minimum elbow room */

    /* Col and Row arrays */
    c = CCOLAMD_C (n_col, ok) ;	    /* size of column structures */
    r = CCOLAMD_R (n_row, ok) ;	    /* size of row structures */
    s = t_add (s, c, ok) ;
    s = t_add (s, r, ok) ;

    c = t_mult (n_col, 3, ok) ;	    /* 3*n_col + 1 */
    c = t_add (c, 1, ok) ;
    s = t_add (s, c, ok) ;

    c = t_add (n_col, 1, ok) ;	    /* 5 * (n_col + 1) */
    c = t_mult (c, 5, ok) ;
    s = t_add (s, c, ok) ;

    s = t_add (s, n_row, ok) ;	    /* n_row */

    return (ok ? s : 0) ;
}

PUBLIC size_t CCOLAMD_recommended	/* returns recommended value of Alen. */
(
    /* === Parameters ======================================================= */

    Int nnz,			/* number of nonzeros in A */
    Int n_row,			/* number of rows in A */
    Int n_col			/* number of columns in A */
)
{
    size_t s ;
    int ok = TRUE ;
    if (nnz < 0 || n_row < 0 || n_col < 0)
    {
	return (0) ;
    }
    s = ccolamd_need (nnz, n_row, n_col, &ok) ;	/* bare minimum needed */
    s = t_add (s, nnz/5, &ok) ;			/* extra elbow room */
    ok = ok && (s < Int_MAX) ;
    return (ok ? s : 0) ;
}


/* ========================================================================== */
/* === ccolamd_set_defaults ================================================= */
/* ========================================================================== */

/*
 *  The ccolamd_set_defaults routine sets the default values of the user-
 *  controllable parameters for ccolamd.
 */

PUBLIC void CCOLAMD_set_defaults
(
    /* === Parameters ======================================================= */

    double knobs [CCOLAMD_KNOBS]		/* knob array */
)
{
    /* === Local variables ================================================== */

    Int i ;

    if (!knobs)
    {
	return ;			/* no knobs to initialize */
    }
    for (i = 0 ; i < CCOLAMD_KNOBS ; i++)
    {
	knobs [i] = 0 ;
    }
    knobs [CCOLAMD_DENSE_ROW] = 10 ;
    knobs [CCOLAMD_DENSE_COL] = 10 ;
    knobs [CCOLAMD_AGGRESSIVE] = TRUE ;	/* default: do aggressive absorption*/
    knobs [CCOLAMD_LU] = FALSE ;	/* default: order for Cholesky */
}


/* ========================================================================== */
/* === symamd =============================================================== */
/* ========================================================================== */

PUBLIC Int CSYMAMD_MAIN		/* return TRUE if OK, FALSE otherwise */
(
    /* === Parameters ======================================================= */

    Int n,				/* number of rows and columns of A */
    Int A [ ],				/* row indices of A */
    Int p [ ],				/* column pointers of A */
    Int perm [ ],			/* output permutation, size n+1 */
    double knobs [CCOLAMD_KNOBS],	/* parameters (uses defaults if NULL) */
    Int stats [CCOLAMD_STATS],		/* output statistics and error codes */
    void * (*allocate) (size_t, size_t),/* pointer to calloc (ANSI C) or */
					/* mxCalloc (for MATLAB mexFunction) */
    void (*release) (void *),		/* pointer to free (ANSI C) or */
    					/* mxFree (for MATLAB mexFunction) */
    Int cmember [ ],			/* constraint set */
    Int stype			        /* stype of A */
)
{
    /* === Local variables ================================================== */

    double cknobs [CCOLAMD_KNOBS] ;
    double default_knobs [CCOLAMD_KNOBS] ;

    Int *count ;		/* length of each column of M, and col pointer*/
    Int *mark ;			/* mark array for finding duplicate entries */
    Int *M ;			/* row indices of matrix M */
    size_t Mlen ;		/* length of M */
    Int n_row ;			/* number of rows in M */
    Int nnz ;			/* number of entries in A */
    Int i ;			/* row index of A */
    Int j ;			/* column index of A */
    Int k ;			/* row index of M */
    Int mnz ;			/* number of nonzeros in M */
    Int pp ;			/* index into a column of A */
    Int last_row ;		/* last row seen in the current column */
    Int length ;		/* number of nonzeros in a column */
    Int both ;			/* TRUE if ordering A+A' */
    Int upper ;			/* TRUE if ordering triu(A)+triu(A)' */
    Int lower ;			/* TRUE if ordering tril(A)+tril(A)' */

#ifndef NDEBUG
    ccolamd_get_debug ("csymamd") ;
#endif

    both = (stype == 0) ;
    upper = (stype > 0) ;
    lower = (stype < 0) ;

    /* === Check the input arguments ======================================== */

    if (!stats)
    {
	DEBUG1 (("csymamd: stats not present\n")) ;
	return (FALSE) ;
    }
    for (i = 0 ; i < CCOLAMD_STATS ; i++)
    {
	stats [i] = 0 ;
    }
    stats [CCOLAMD_STATUS] = CCOLAMD_OK ;
    stats [CCOLAMD_INFO1] = -1 ;
    stats [CCOLAMD_INFO2] = -1 ;

    if (!A)
    {
    	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_A_not_present ;
	DEBUG1 (("csymamd: A not present\n")) ;
	return (FALSE) ;
    }

    if (!p)		/* p is not present */
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_p_not_present ;
	DEBUG1 (("csymamd: p not present\n")) ;
    	return (FALSE) ;
    }

    if (n < 0)		/* n must be >= 0 */
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_ncol_negative ;
	stats [CCOLAMD_INFO1] = n ;
	DEBUG1 (("csymamd: n negative "ID" \n", n)) ;
    	return (FALSE) ;
    }

    nnz = p [n] ;
    if (nnz < 0)	/* nnz must be >= 0 */
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_nnz_negative ;
	stats [CCOLAMD_INFO1] = nnz ;
	DEBUG1 (("csymamd: number of entries negative "ID" \n", nnz)) ;
	return (FALSE) ;
    }

    if (p [0] != 0)
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_p0_nonzero ;
	stats [CCOLAMD_INFO1] = p [0] ;
	DEBUG1 (("csymamd: p[0] not zero "ID"\n", p [0])) ;
	return (FALSE) ;
    }

    /* === If no knobs, set default knobs =================================== */

    if (!knobs)
    {
	CCOLAMD_set_defaults (default_knobs) ;
	knobs = default_knobs ;
    }

    /* === Allocate count and mark ========================================== */

    count = (Int *) ((*allocate) (n+1, sizeof (Int))) ;
    if (!count)
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_out_of_memory ;
	DEBUG1 (("csymamd: allocate count (size "ID") failed\n", n+1)) ;
	return (FALSE) ;
    }

    mark = (Int *) ((*allocate) (n+1, sizeof (Int))) ;
    if (!mark)
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_out_of_memory ;
	(*release) ((void *) count) ;
	DEBUG1 (("csymamd: allocate mark (size "ID") failed\n", n+1)) ;
	return (FALSE) ;
    }

    /* === Compute column counts of M, check if A is valid ================== */

    stats [CCOLAMD_INFO3] = 0 ; /* number of duplicate or unsorted row indices*/

    for (i = 0 ; i < n ; i++)
    {
    	mark [i] = -1 ;
    }

    for (j = 0 ; j < n ; j++)
    {
	last_row = -1 ;

	length = p [j+1] - p [j] ;
	if (length < 0)
	{
	    /* column pointers must be non-decreasing */
	    stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_col_length_negative ;
	    stats [CCOLAMD_INFO1] = j ;
	    stats [CCOLAMD_INFO2] = length ;
	    (*release) ((void *) count) ;
	    (*release) ((void *) mark) ;
	    DEBUG1 (("csymamd: col "ID" negative length "ID"\n", j, length)) ;
	    return (FALSE) ;
	}

	for (pp = p [j] ; pp < p [j+1] ; pp++)
	{
	    i = A [pp] ;
	    if (i < 0 || i >= n)
	    {
		/* row index i, in column j, is out of bounds */
		stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_row_index_out_of_bounds ;
		stats [CCOLAMD_INFO1] = j ;
		stats [CCOLAMD_INFO2] = i ;
		stats [CCOLAMD_INFO3] = n ;
		(*release) ((void *) count) ;
		(*release) ((void *) mark) ;
		DEBUG1 (("csymamd: row "ID" col "ID" out of bounds\n", i, j)) ;
		return (FALSE) ;
	    }

	    if (i <= last_row || mark [i] == j)
	    {
		/* row index is unsorted or repeated (or both), thus col */
		/* is jumbled.  This is a notice, not an error condition. */
		stats [CCOLAMD_STATUS] = CCOLAMD_OK_BUT_JUMBLED ;
		stats [CCOLAMD_INFO1] = j ;
		stats [CCOLAMD_INFO2] = i ;
		(stats [CCOLAMD_INFO3]) ++ ;
		DEBUG1 (("csymamd: row "ID" col "ID" unsorted/dupl.\n", i, j)) ;
	    }

	    if (mark [i] != j)
	    {
		if ((both && i != j) || (lower && i > j) || (upper && i < j))
		{
		    /* row k of M will contain column indices i and j */
		    count [i]++ ;
		    count [j]++ ;
		}
	    }

	    /* mark the row as having been seen in this column */
	    mark [i] = j ;

	    last_row = i ;
	}
    }

    /* === Compute column pointers of M ===================================== */

    /* use output permutation, perm, for column pointers of M */
    perm [0] = 0 ;
    for (j = 1 ; j <= n ; j++)
    {
	perm [j] = perm [j-1] + count [j-1] ;
    }
    for (j = 0 ; j < n ; j++)
    {
	count [j] = perm [j] ;
    }

    /* === Construct M ====================================================== */

    mnz = perm [n] ;
    n_row = mnz / 2 ;
    Mlen = CCOLAMD_recommended (mnz, n_row, n) ;
    M = (Int *) ((*allocate) (Mlen, sizeof (Int))) ;
    DEBUG1 (("csymamd: M is "ID"-by-"ID" with "ID" entries, Mlen = %g\n",
    	n_row, n, mnz, (double) Mlen)) ;

    if (!M)
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_out_of_memory ;
	(*release) ((void *) count) ;
	(*release) ((void *) mark) ;
	DEBUG1 (("csymamd: allocate M (size %g) failed\n", (double) Mlen)) ;
	return (FALSE) ;
    }

    k = 0 ;

    if (stats [CCOLAMD_STATUS] == CCOLAMD_OK)
    {
	/* Matrix is OK */
	for (j = 0 ; j < n ; j++)
	{
	    ASSERT (p [j+1] - p [j] >= 0) ;
	    for (pp = p [j] ; pp < p [j+1] ; pp++)
	    {
		i = A [pp] ;
		ASSERT (i >= 0 && i < n) ;
		if ((both && i != j) || (lower && i > j) || (upper && i < j))
		{
		    /* row k of M contains column indices i and j */
		    M [count [i]++] = k ;
		    M [count [j]++] = k ;
		    k++ ;
		}
	    }
	}
    }
    else
    {
	/* Matrix is jumbled.  Do not add duplicates to M.  Unsorted cols OK. */
	DEBUG1 (("csymamd: Duplicates in A.\n")) ;
	for (i = 0 ; i < n ; i++)
	{
	    mark [i] = -1 ;
	}
	for (j = 0 ; j < n ; j++)
	{
	    ASSERT (p [j+1] - p [j] >= 0) ;
	    for (pp = p [j] ; pp < p [j+1] ; pp++)
	    {
		i = A [pp] ;
		ASSERT (i >= 0 && i < n) ;
		if (mark [i] != j)
		{
		    if ((both && i != j) || (lower && i > j) || (upper && i<j))
		    {
			/* row k of M contains column indices i and j */
			M [count [i]++] = k ;
			M [count [j]++] = k ;
			k++ ;
			mark [i] = j ;
		    }
		}
	    }
	}
    }

    /* count and mark no longer needed */
    (*release) ((void *) mark) ;
    (*release) ((void *) count) ;
    ASSERT (k == n_row) ;

    /* === Adjust the knobs for M =========================================== */

    for (i = 0 ; i < CCOLAMD_KNOBS ; i++)
    {
	cknobs [i] = knobs [i] ;
    }

    /* there are no dense rows in M */
    cknobs [CCOLAMD_DENSE_ROW] = -1 ;
    cknobs [CCOLAMD_DENSE_COL] = knobs [CCOLAMD_DENSE_ROW] ;

    /* ensure CCSYMAMD orders for Cholesky, not LU */
    cknobs [CCOLAMD_LU] = FALSE ;

    /* === Order the columns of M =========================================== */

    (void) CCOLAMD_2 (n_row, n, (Int) Mlen, M, perm, cknobs, stats,
             (Int *) NULL, (Int *) NULL, (Int *) NULL, (Int *) NULL,
             (Int *) NULL, (Int *) NULL, (Int *) NULL, cmember) ;

    /* === adjust statistics ================================================ */

    /* a dense column in ccolamd means a dense row and col in csymamd */
    stats [CCOLAMD_DENSE_ROW] = stats [CCOLAMD_DENSE_COL] ;

    /* === Free M =========================================================== */

    (*release) ((void *) M) ;
    DEBUG1 (("csymamd: done.\n")) ;
    return (TRUE) ;
}


/* ========================================================================== */
/* === ccolamd ============================================================== */
/* ========================================================================== */

/*
 *  The colamd routine computes a column ordering Q of a sparse matrix
 *  A such that the LU factorization P(AQ) = LU remains sparse, where P is
 *  selected via partial pivoting.   The routine can also be viewed as
 *  providing a permutation Q such that the Cholesky factorization
 *  (AQ)'(AQ) = LL' remains sparse.
 */

PUBLIC Int CCOLAMD_MAIN
(
    /* === Parameters ======================================================= */

    Int n_row,			/* number of rows in A */
    Int n_col,			/* number of columns in A */
    Int Alen,			/* length of A */
    Int A [ ],			/* row indices of A */
    Int p [ ],			/* pointers to columns in A */
    double knobs [CCOLAMD_KNOBS],/* parameters (uses defaults if NULL) */
    Int stats [CCOLAMD_STATS],	/* output statistics and error codes */
    Int cmember [ ]		/* constraint set of A */
)
{
     return (CCOLAMD_2 (n_row, n_col, Alen, A, p, knobs, stats,
             (Int *) NULL, (Int *) NULL, (Int *) NULL, (Int *) NULL,
             (Int *) NULL, (Int *) NULL, (Int *) NULL, cmember)) ;
}


/* ========================================================================== */
/* === ccolamd2 ============================================================= */
/* ========================================================================== */

/* Identical to ccolamd, except that additional information about each frontal
 * matrix is returned to the caller.  Not intended to be directly called by
 * the user.
 */

PUBLIC Int CCOLAMD_2	    /* returns TRUE if successful, FALSE otherwise */
(
    /* === Parameters ======================================================= */

    Int n_row,			/* number of rows in A */
    Int n_col,			/* number of columns in A */
    Int Alen,			/* length of A */
    Int A [ ],			/* row indices of A */
    Int p [ ],			/* pointers to columns in A */
    double knobs [CCOLAMD_KNOBS],/* parameters (uses defaults if NULL) */
    Int stats [CCOLAMD_STATS],	/* output statistics and error codes */

    /* each Front array is of size n_col+1. */
    Int Front_npivcol [ ],	/* # pivot cols in each front */
    Int Front_nrows [ ],	/* # of rows in each front (incl. pivot rows) */
    Int Front_ncols [ ],	/* # of cols in each front (incl. pivot cols) */
    Int Front_parent [ ],	/* parent of each front */
    Int Front_cols [ ],		/* link list of pivot columns for each front */
    Int *p_nfr,			/* total number of frontal matrices */
    Int InFront [ ],		/* InFront [row] = f if the original row was
				 * absorbed into front f.  EMPTY if the row was
				 * empty, dense, or not absorbed.  This array
				 * has size n_row+1 */
    Int cmember [ ]		/* constraint set of A */
)
{
    /* === Local variables ================================================== */

    Int i ;			/* loop index */
    Int nnz ;			/* nonzeros in A */
    size_t Row_size ;		/* size of Row [ ], in integers */
    size_t Col_size ;		/* size of Col [ ], in integers */
    size_t need ;		/* minimum required length of A */
    CColamd_Row *Row ;		/* pointer into A of Row [0..n_row] array */
    CColamd_Col *Col ;		/* pointer into A of Col [0..n_col] array */
    Int n_col2 ;		/* number of non-dense, non-empty columns */
    Int n_row2 ;		/* number of non-dense, non-empty rows */
    Int ngarbage ;		/* number of garbage collections performed */
    Int max_deg ;		/* maximum row degree */
    double default_knobs [CCOLAMD_KNOBS] ;	/* default knobs array */

    Int n_cset ;		/* number of constraint sets */
    Int *cset ;			/* cset of A */
    Int *cset_start ;		/* pointer into cset */
    Int *temp_cstart ;		/* temp pointer to start of cset */
    Int *csize ;		/* temp pointer to cset size */
    Int ap ;			/* column index */
    Int order_for_lu ;		/* TRUE: order for LU, FALSE: for Cholesky */

    Int ndense_row, nempty_row, parent, ndense_col,
    	nempty_col, k, col, nfr, *Front_child, *Front_sibling, *Front_stack,
    	*Front_order, *Front_size ;
    Int nnewlyempty_col, nnewlyempty_row ;
    Int aggressive ;
    Int row ;
    Int *dead_cols ;
    Int set1 ;
    Int set2 ;
    Int cs ;

    int ok ;

#ifndef NDEBUG
    ccolamd_get_debug ("ccolamd") ;
#endif

    /* === Check the input arguments ======================================== */

    if (!stats)
    {
	DEBUG1 (("ccolamd: stats not present\n")) ;
	return (FALSE) ;
    }
    for (i = 0 ; i < CCOLAMD_STATS ; i++)
    {
	stats [i] = 0 ;
    }
    stats [CCOLAMD_STATUS] = CCOLAMD_OK ;
    stats [CCOLAMD_INFO1] = -1 ;
    stats [CCOLAMD_INFO2] = -1 ;

    if (!A)		/* A is not present */
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_A_not_present ;
	DEBUG1 (("ccolamd: A not present\n")) ;
	return (FALSE) ;
    }

    if (!p)		/* p is not present */
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_p_not_present ;
	DEBUG1 (("ccolamd: p not present\n")) ;
    	return (FALSE) ;
    }

    if (n_row < 0)	/* n_row must be >= 0 */
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_nrow_negative ;
	stats [CCOLAMD_INFO1] = n_row ;
	DEBUG1 (("ccolamd: nrow negative "ID"\n", n_row)) ;
    	return (FALSE) ;
    }

    if (n_col < 0)	/* n_col must be >= 0 */
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_ncol_negative ;
	stats [CCOLAMD_INFO1] = n_col ;
	DEBUG1 (("ccolamd: ncol negative "ID"\n", n_col)) ;
    	return (FALSE) ;
    }

    nnz = p [n_col] ;
    if (nnz < 0)	/* nnz must be >= 0 */
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_nnz_negative ;
	stats [CCOLAMD_INFO1] = nnz ;
	DEBUG1 (("ccolamd: number of entries negative "ID"\n", nnz)) ;
	return (FALSE) ;
    }

    if (p [0] != 0)
    {
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_p0_nonzero ;
	stats [CCOLAMD_INFO1] = p [0] ;
	DEBUG1 (("ccolamd: p[0] not zero "ID"\n", p [0])) ;
	return (FALSE) ;
    }

    /* === If no knobs, set default knobs =================================== */

    if (!knobs)
    {
	CCOLAMD_set_defaults (default_knobs) ;
	knobs = default_knobs ;
    }

    aggressive = (knobs [CCOLAMD_AGGRESSIVE] != FALSE) ;
    order_for_lu = (knobs [CCOLAMD_LU] != FALSE) ;

    /* === Allocate workspace from array A ================================== */

    ok = TRUE ;
    Col_size = CCOLAMD_C (n_col, &ok) ;
    Row_size = CCOLAMD_R (n_row, &ok) ;

    /* min size of A is 2nnz+ncol.  cset and cset_start are of size 2ncol+1 */
    /* Each of the 5 fronts is of size n_col + 1. InFront is of size nrow.  */

    /*
    need = MAX (2*nnz, 4*n_col) + n_col +
    		Col_size + Row_size +
		(3*n_col+1) + (5*(n_col+1)) + n_row ;
    */
    need = ccolamd_need (nnz, n_row, n_col, &ok) ;

    if (!ok || need > (size_t) Alen || need > Int_MAX)
    {
	/* not enough space in array A to perform the ordering */
	stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_A_too_small ;
	stats [CCOLAMD_INFO1] = need ;
	stats [CCOLAMD_INFO2] = Alen ;
	DEBUG1 (("ccolamd: Need Alen >= "ID", given "ID"\n", need, Alen)) ;
	return (FALSE) ;
    }

    /* since integer overflow has been check, the following cannot overflow: */
    Alen -= Col_size + Row_size + (3*n_col + 1) + 5*(n_col+1) + n_row ;

    /* Size of A is now Alen >= MAX (2*nnz, 4*n_col) + n_col.  The ordering
     * requires Alen >= 2*nnz + n_col, and the postorder requires
     * Alen >= 5*n_col. */

    ap = Alen ;

    /* Front array workspace: 5*(n_col+1) + n_row */
    if (!Front_npivcol || !Front_nrows || !Front_ncols || !Front_parent ||
        !Front_cols || !Front_cols || !InFront)
    {
	Front_npivcol = &A [ap] ; ap += (n_col + 1) ;
	Front_nrows = &A [ap] ;   ap += (n_col + 1) ;
	Front_ncols = &A [ap] ;   ap += (n_col + 1) ;
	Front_parent = &A [ap] ;  ap += (n_col + 1) ;
	Front_cols = &A [ap] ;	  ap += (n_col + 1) ;
	InFront = &A [ap] ;	  ap += (n_row) ;
    }
    else
    {
	/* Fronts are present. Leave the additional space as elbow room. */
    	ap += 5*(n_col+1) + n_row ;
	ap = Alen ;
    }

    /* Workspace for cset management: 3*n_col+1 */
    /* cset_start is of size n_col + 1 */
    cset_start = &A [ap] ;
    ap += n_col + 1 ;

    /* dead_col is of size n_col */
    dead_cols = &A [ap] ;
    ap += n_col ;

    /* cset is of size n_col */
    cset = &A [ap] ;
    ap += n_col ;

    /* Col is of size Col_size.  The space is shared by temp_cstart and csize */
    Col = (CColamd_Col *) &A [ap] ;
    temp_cstart = (Int *) Col ;		/* [ temp_cstart is of size n_col+1 */
    csize = temp_cstart + (n_col+1) ;	/* csize is of size n_col+1 */
    ap += Col_size ;
    ASSERT (Col_size >= 2*n_col+1) ;

    /* Row is of size Row_size */
    Row = (CColamd_Row *) &A [ap] ;
    ap += Row_size ;

    /* Initialize csize & dead_cols to zero */
    for (i = 0 ; i < n_col ; i++)
    {
    	csize [i] = 0 ;
	dead_cols [i] = 0 ;
    }

    /* === Construct the constraint set ===================================== */

    if (n_col == 0)
    {
	n_cset = 0 ;
    }
    else if (cmember == (Int *) NULL)
    {
	/* no constraint set; all columns belong to set zero */
	n_cset = 1 ;
	csize [0] = n_col ;
	DEBUG1 (("no cmember present\n")) ;
    }
    else
    {
	n_cset = 0 ;
	for (i = 0 ; i < n_col ; i++)
	{
	    if (cmember [i] < 0 || cmember [i] > n_col)
	    {
		stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_invalid_cmember ;
		DEBUG1 (("ccolamd: malformed cmember \n")) ;
		return (FALSE) ;
	    }
	    n_cset = MAX (n_cset, cmember [i]) ;
	    csize [cmember [i]]++ ;
	}
	/* cset is zero based */
	n_cset++ ;
    }

    ASSERT ((n_cset >= 0) && (n_cset <= n_col)) ;

    cset_start [0] = temp_cstart [0] = 0 ;
    for (i = 1 ; i <= n_cset ; i++)
    {
	cset_start [i] = cset_start [i-1] + csize [i-1] ;
	DEBUG4 ((" cset_start ["ID"] = "ID" \n", i , cset_start [i])) ;
	temp_cstart [i] = cset_start [i] ;
    }

    /* do in reverse order to encourage natural tie-breaking */
    if (cmember == (Int *) NULL)
    {
	for (i = n_col-1 ; i >= 0 ; i--)
	{
	    cset [temp_cstart [0]++] = i ;
	}
    }
    else
    {
	for (i = n_col-1 ; i >= 0 ; i--)
	{
	    cset [temp_cstart [cmember [i]]++] = i ;
	}
    }

    /* ] temp_cstart and csize are no longer used */

    /* === Construct the row and column data structures ===================== */

    if (!init_rows_cols (n_row, n_col, Row, Col, A, p, stats))
    {
	/* input matrix is invalid */
	DEBUG1 (("ccolamd: Matrix invalid\n")) ;
	return (FALSE) ;
    }

    /* === Initialize front info ============================================ */

    for (col = 0 ; col < n_col ; col++)
    {
    	Front_npivcol [col] = 0 ;
    	Front_nrows [col] = 0 ;
    	Front_ncols [col] = 0 ;
    	Front_parent [col] = EMPTY ;
    	Front_cols [col] = EMPTY ;
    }

    /* === Initialize scores, kill dense rows/columns ======================= */

    init_scoring (n_row, n_col, Row, Col, A, p, knobs,
	&n_row2, &n_col2, &max_deg, cmember, n_cset, cset_start, dead_cols,
	&ndense_row, &nempty_row, &nnewlyempty_row,
	&ndense_col, &nempty_col, &nnewlyempty_col) ;

    ASSERT (n_row2 == n_row - nempty_row - nnewlyempty_row - ndense_row) ;
    ASSERT (n_col2 == n_col - nempty_col - nnewlyempty_col - ndense_col) ;
    DEBUG1 (("# dense rows "ID" cols "ID"\n", ndense_row, ndense_col)) ;

    /* === Order the supercolumns =========================================== */

    ngarbage = find_ordering (n_row, n_col, Alen, Row, Col, A, p,
#ifndef NDEBUG
	n_col2,
#endif
	max_deg, 2*nnz, cset, cset_start,
#ifndef NDEBUG
	n_cset,
#endif
	cmember, Front_npivcol, Front_nrows, Front_ncols, Front_parent,
	Front_cols, &nfr, aggressive, InFront, order_for_lu) ;

    ASSERT (Alen >= 5*n_col) ;

    /* === Postorder ======================================================== */

    /* A is no longer needed, so use A [0..5*nfr-1] as workspace [ [ */
    /* This step requires Alen >= 5*n_col */
    Front_child   = A ;
    Front_sibling = Front_child + nfr ;
    Front_stack   = Front_sibling + nfr ;
    Front_order   = Front_stack + nfr ;
    Front_size    = Front_order + nfr ;

    CCOLAMD_fsize (nfr, Front_size, Front_nrows, Front_ncols,
            Front_parent, Front_npivcol) ;

    CCOLAMD_postorder (nfr, Front_parent, Front_npivcol, Front_size,
        Front_order, Front_child, Front_sibling, Front_stack, Front_cols,
	cmember) ;

    /* Front_size, Front_stack, Front_child, Front_sibling no longer needed ] */

    /* use A [0..nfr-1] as workspace */
    CCOLAMD_apply_order (Front_npivcol, Front_order, A, nfr, nfr) ;
    CCOLAMD_apply_order (Front_nrows,   Front_order, A, nfr, nfr) ;
    CCOLAMD_apply_order (Front_ncols,   Front_order, A, nfr, nfr) ;
    CCOLAMD_apply_order (Front_parent,  Front_order, A, nfr, nfr) ;
    CCOLAMD_apply_order (Front_cols,    Front_order, A, nfr, nfr) ;

    /* fix the parent to refer to the new numbering */
    for (i = 0 ; i < nfr ; i++)
    {
        parent = Front_parent [i] ;
        if (parent != EMPTY)
        {
            Front_parent [i] = Front_order [parent] ;
        }
    }

    /* fix InFront to refer to the new numbering */
    for (row = 0 ; row < n_row ; row++)
    {
        i = InFront [row] ;
        ASSERT (i >= EMPTY && i < nfr) ;
        if (i != EMPTY)
        {
            InFront [row] = Front_order [i] ;
        }
    }

    /* Front_order longer needed ] */

    /* === Order the columns in the fronts ================================== */

    /* use A [0..n_col-1] as inverse permutation */
    for (i = 0 ; i < n_col ; i++)
    {
        A [i] = EMPTY ;
    }

    k = 0 ;
    set1 = 0 ;
    for (i = 0 ; i < nfr ; i++)
    {
        ASSERT (Front_npivcol [i] > 0) ;

	set2 = CMEMBER (Front_cols [i]) ;
        while (set1 < set2)
        {
            k += dead_cols [set1] ;
            DEBUG3 (("Skip null/dense columns of set "ID"\n",set1)) ;
            set1++ ;
        }
        set1 = set2 ;

        for (col = Front_cols [i] ; col != EMPTY ; col = Col [col].nextcol)
        {
            ASSERT (col >= 0 && col < n_col) ;
            DEBUG1 (("ccolamd output ordering: k "ID" col "ID"\n", k, col)) ;
            p [k] = col ;
            ASSERT (A [col] == EMPTY) ;

	    cs = CMEMBER (col) ;
            ASSERT (k >= cset_start [cs] && k < cset_start [cs+1]) ;

            A [col] = k ;
            k++ ;
        }
    }

    /* === Order the "dense" and null columns =============================== */

    if (n_col2 < n_col)
    {
        for (col = 0 ; col < n_col ; col++)
        {
            if (A [col] == EMPTY)
            {
                k = Col [col].shared2.order ;
		cs = CMEMBER (col) ;
#ifndef NDEBUG
                dead_cols [cs]-- ;
#endif
                ASSERT (k >= cset_start [cs] && k < cset_start [cs+1]) ;
                DEBUG1 (("ccolamd output ordering: k "ID" col "ID
                    " (dense or null col)\n", k, col)) ;
                p [k] = col ;
                A [col] = k ;
            }
        }
    }

#ifndef NDEBUG
    for (i = 0 ; i < n_cset ; i++)
    {
    	ASSERT (dead_cols [i] == 0) ;
    }
#endif

    /* === Return statistics in stats ======================================= */

    stats [CCOLAMD_DENSE_ROW] = ndense_row ;
    stats [CCOLAMD_DENSE_COL] = nempty_row ;
    stats [CCOLAMD_NEWLY_EMPTY_ROW] = nnewlyempty_row ;
    stats [CCOLAMD_DENSE_COL] = ndense_col ;
    stats [CCOLAMD_EMPTY_COL] = nempty_col ;
    stats [CCOLAMD_NEWLY_EMPTY_COL] = nnewlyempty_col ;
    ASSERT (ndense_col + nempty_col + nnewlyempty_col == n_col - n_col2) ;
    if (p_nfr)
    {
    	*p_nfr = nfr ;
    }
    stats [CCOLAMD_DEFRAG_COUNT] = ngarbage ;
    DEBUG1 (("ccolamd: done.\n")) ;
    return (TRUE) ;
}


/* ========================================================================== */
/* === colamd_report ======================================================== */
/* ========================================================================== */

PUBLIC void CCOLAMD_report
(
    Int stats [CCOLAMD_STATS]
)
{
    print_report ("ccolamd", stats) ;
}


/* ========================================================================== */
/* === symamd_report ======================================================== */
/* ========================================================================== */

PUBLIC void CSYMAMD_report
(
    Int stats [CCOLAMD_STATS]
)
{
    print_report ("csymamd", stats) ;
}


/* ========================================================================== */
/* === NON-USER-CALLABLE ROUTINES: ========================================== */
/* ========================================================================== */

/* There are no user-callable routines beyond this point in the file */


/* ========================================================================== */
/* === init_rows_cols ======================================================= */
/* ========================================================================== */

/*
    Takes the column form of the matrix in A and creates the row form of the
    matrix.  Also, row and column attributes are stored in the Col and Row
    structs.  If the columns are un-sorted or contain duplicate row indices,
    this routine will also sort and remove duplicate row indices from the
    column form of the matrix.  Returns FALSE if the matrix is invalid,
    TRUE otherwise.  Not user-callable.
*/

PRIVATE Int init_rows_cols	/* returns TRUE if OK, or FALSE otherwise */
(
    /* === Parameters ======================================================= */

    Int n_row,			/* number of rows of A */
    Int n_col,			/* number of columns of A */
    CColamd_Row Row [ ],		/* of size n_row+1 */
    CColamd_Col Col [ ],		/* of size n_col+1 */
    Int A [ ],			/* row indices of A, of size Alen */
    Int p [ ],			/* pointers to columns in A, of size n_col+1 */
    Int stats [CCOLAMD_STATS]	/* colamd statistics */
)
{
    /* === Local variables ================================================== */

    Int col ;			/* a column index */
    Int row ;			/* a row index */
    Int *cp ;			/* a column pointer */
    Int *cp_end ;		/* a pointer to the end of a column */
    Int *rp ;			/* a row pointer */
    Int *rp_end ;		/* a pointer to the end of a row */
    Int last_row ;		/* previous row */

    /* === Initialize columns, and check column pointers ==================== */

    for (col = 0 ; col < n_col ; col++)
    {
	Col [col].start = p [col] ;
	Col [col].length = p [col+1] - p [col] ;

	if (Col [col].length < 0)
	{
	    /* column pointers must be non-decreasing */
	    stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_col_length_negative ;
	    stats [CCOLAMD_INFO1] = col ;
	    stats [CCOLAMD_INFO2] = Col [col].length ;
	    DEBUG1 (("ccolamd: col "ID" length "ID" < 0\n",
			col, Col [col].length)) ;
	    return (FALSE) ;
	}

	Col [col].shared1.thickness = 1 ;
	Col [col].shared2.score = 0 ;
	Col [col].shared3.prev = EMPTY ;
	Col [col].shared4.degree_next = EMPTY ;
        Col [col].nextcol = EMPTY ;
        Col [col].lastcol = col ;
    }

    /* p [0..n_col] no longer needed, used as "head" in subsequent routines */

    /* === Scan columns, compute row degrees, and check row indices ========= */

    stats [CCOLAMD_INFO3] = 0 ;	/* number of duplicate or unsorted row indices*/

    for (row = 0 ; row < n_row ; row++)
    {
	Row [row].length = 0 ;
	Row [row].shared2.mark = -1 ;
        Row [row].thickness = 1 ;
        Row [row].front = EMPTY ;
    }

    for (col = 0 ; col < n_col ; col++)
    {
	DEBUG1 (("\nCcolamd input column "ID":\n", col)) ;
	last_row = -1 ;

	cp = &A [p [col]] ;
	cp_end = &A [p [col+1]] ;

	while (cp < cp_end)
	{
	    row = *cp++ ;
	    DEBUG1 (("row: "ID"\n", row)) ;

	    /* make sure row indices within range */
	    if (row < 0 || row >= n_row)
	    {
		stats [CCOLAMD_STATUS] = CCOLAMD_ERROR_row_index_out_of_bounds ;
		stats [CCOLAMD_INFO1] = col ;
		stats [CCOLAMD_INFO2] = row ;
		stats [CCOLAMD_INFO3] = n_row ;
		DEBUG1 (("row "ID" col "ID" out of bounds\n", row, col)) ;
		return (FALSE) ;
	    }

	    if (row <= last_row || Row [row].shared2.mark == col)
	    {
		/* row index are unsorted or repeated (or both), thus col */
		/* is jumbled.  This is a notice, not an error condition. */
		stats [CCOLAMD_STATUS] = CCOLAMD_OK_BUT_JUMBLED ;
		stats [CCOLAMD_INFO1] = col ;
		stats [CCOLAMD_INFO2] = row ;
		(stats [CCOLAMD_INFO3]) ++ ;
		DEBUG1 (("row "ID" col "ID" unsorted/duplicate\n", row, col)) ;
	    }

	    if (Row [row].shared2.mark != col)
	    {
		Row [row].length++ ;
	    }
	    else
	    {
		/* this is a repeated entry in the column, */
		/* it will be removed */
		Col [col].length-- ;
	    }

	    /* mark the row as having been seen in this column */
	    Row [row].shared2.mark = col ;

	    last_row = row ;
	}
    }

    /* === Compute row pointers ============================================= */

    /* row form of the matrix starts directly after the column */
    /* form of matrix in A */
    Row [0].start = p [n_col] ;
    Row [0].shared1.p = Row [0].start ;
    Row [0].shared2.mark = -1 ;
    for (row = 1 ; row < n_row ; row++)
    {
	Row [row].start = Row [row-1].start + Row [row-1].length ;
	Row [row].shared1.p = Row [row].start ;
	Row [row].shared2.mark = -1 ;
    }

    /* === Create row form ================================================== */

    if (stats [CCOLAMD_STATUS] == CCOLAMD_OK_BUT_JUMBLED)
    {
	/* if cols jumbled, watch for repeated row indices */
 	for (col = 0 ; col < n_col ; col++)
	{
	    cp = &A [p [col]] ;
	    cp_end = &A [p [col+1]] ;
	    while (cp < cp_end)
	    {
		row = *cp++ ;
		if (Row [row].shared2.mark != col)
		{
		    A [(Row [row].shared1.p)++] = col ;
		    Row [row].shared2.mark = col ;
		}
	    }
	}
    }
    else
    {
	/* if cols not jumbled, we don't need the mark (this is faster) */
	for (col = 0 ; col < n_col ; col++)
	{
	    cp = &A [p [col]] ;
	    cp_end = &A [p [col+1]] ;
	    while (cp < cp_end)
	    {
		A [(Row [*cp++].shared1.p)++] = col ;
	    }
	}
    }

    /* === Clear the row marks and set row degrees ========================== */

    for (row = 0 ; row < n_row ; row++)
    {
	Row [row].shared2.mark = 0 ;
	Row [row].shared1.degree = Row [row].length ;
    }

    /* === See if we need to re-create columns ============================== */

    if (stats [CCOLAMD_STATUS] == CCOLAMD_OK_BUT_JUMBLED)
    {
    	DEBUG1 (("ccolamd: reconstructing column form, matrix jumbled\n")) ;

#ifndef NDEBUG
	/* make sure column lengths are correct */
 	for (col = 0 ; col < n_col ; col++)
	{
	    p [col] = Col [col].length ;
	}
	for (row = 0 ; row < n_row ; row++)
	{
	    rp = &A [Row [row].start] ;
	    rp_end = rp + Row [row].length ;
	    while (rp < rp_end)
	    {
		p [*rp++]-- ;
	    }
	}
	for (col = 0 ; col < n_col ; col++)
	{
	    ASSERT (p [col] == 0) ;
	}
	/* now p is all zero (different than when debugging is turned off) */
#endif

	/* === Compute col pointers ========================================= */

	/* col form of the matrix starts at A [0]. */
	/* Note, we may have a gap between the col form and the row */
	/* form if there were duplicate entries, if so, it will be */
	/* removed upon the first garbage collection */
	Col [0].start = 0 ;
	p [0] = Col [0].start ;
	for (col = 1 ; col < n_col ; col++)
	{
	    /* note that the lengths here are for pruned columns, i.e. */
	    /* no duplicate row indices will exist for these columns */
	    Col [col].start = Col [col-1].start + Col [col-1].length ;
	    p [col] = Col [col].start ;
	}

	/* === Re-create col form =========================================== */

	for (row = 0 ; row < n_row ; row++)
	{
	    rp = &A [Row [row].start] ;
	    rp_end = rp + Row [row].length ;
	    while (rp < rp_end)
	    {
		A [(p [*rp++])++] = row ;
	    }
	}
    }

    /* === Done.  Matrix is not (or no longer) jumbled ====================== */


    return (TRUE) ;
}


/* ========================================================================== */
/* === init_scoring ========================================================= */
/* ========================================================================== */

/*
    Kills dense or empty columns and rows, calculates an initial score for
    each column, and places all columns in the degree lists.  Not user-callable.
*/

PRIVATE void init_scoring
(
    /* === Parameters ======================================================= */

    Int n_row,			/* number of rows of A */
    Int n_col,			/* number of columns of A */
    CColamd_Row Row [ ],	/* of size n_row+1 */
    CColamd_Col Col [ ],	/* of size n_col+1 */
    Int A [ ],			/* column form and row form of A */
    Int head [ ],		/* of size n_col+1 */
    double knobs [CCOLAMD_KNOBS],/* parameters */
    Int *p_n_row2,		/* number of non-dense, non-empty rows */
    Int *p_n_col2,		/* number of non-dense, non-empty columns */
    Int *p_max_deg,		/* maximum row degree */
    Int cmember [ ],
    Int n_cset,
    Int cset_start [ ],
    Int dead_cols [ ],
    Int *p_ndense_row,		/* number of dense rows */
    Int *p_nempty_row,		/* number of original empty rows */
    Int *p_nnewlyempty_row,	/* number of newly empty rows */
    Int *p_ndense_col,		/* number of dense cols (excl "empty" cols) */
    Int *p_nempty_col,		/* number of original empty cols */
    Int *p_nnewlyempty_col	/* number of newly empty cols */
)
{
/* === Local variables ================================================== */

    Int c ;			/* a column index */
    Int r, row ;		/* a row index */
    Int *cp ;			/* a column pointer */
    Int deg ;			/* degree of a row or column */
    Int *cp_end ;		/* a pointer to the end of a column */
    Int *new_cp ;		/* new column pointer */
    Int col_length ;		/* length of pruned column */
    Int score ;			/* current column score */
    Int n_col2 ;		/* number of non-dense, non-empty columns */
    Int n_row2 ;		/* number of non-dense, non-empty rows */
    Int dense_row_count ;	/* remove rows with more entries than this */
    Int dense_col_count ;	/* remove cols with more entries than this */
    Int max_deg ;		/* maximum row degree */
    Int s ;			/* a cset index */
    Int ndense_row ;		/* number of dense rows */
    Int nempty_row ;		/* number of empty rows */
    Int nnewlyempty_row ;	/* number of newly empty rows */
    Int ndense_col ;		/* number of dense cols (excl "empty" cols) */
    Int nempty_col ;		/* number of original empty cols */
    Int nnewlyempty_col ;	/* number of newly empty cols */
    Int ne ;

#ifndef NDEBUG
    Int debug_count ;		/* debug only. */
#endif

    /* === Extract knobs ==================================================== */

    /* Note: if knobs contains a NaN, this is undefined: */
    if (knobs [CCOLAMD_DENSE_ROW] < 0)
    {
	/* only remove completely dense rows */
	dense_row_count = n_col-1 ;
    }
    else
    {
	dense_row_count = DENSE_DEGREE (knobs [CCOLAMD_DENSE_ROW], n_col) ;
    }
    if (knobs [CCOLAMD_DENSE_COL] < 0)
    {
	/* only remove completely dense columns */
	dense_col_count = n_row-1 ;
    }
    else
    {
	dense_col_count =
	    DENSE_DEGREE (knobs [CCOLAMD_DENSE_COL], MIN (n_row, n_col)) ;
    }

    DEBUG1 (("densecount: "ID" "ID"\n", dense_row_count, dense_col_count)) ;
    max_deg = 0 ;

    n_col2 = n_col ;
    n_row2 = n_row ;

    /* Set the head array for bookkeeping of dense and empty columns. */
    /* This will be used as hash buckets later. */
    for (s = 0 ; s < n_cset ; s++)
    {
	head [s] = cset_start [s+1] ;
    }

    ndense_col = 0 ;
    nempty_col = 0 ;
    nnewlyempty_col = 0 ;
    ndense_row = 0 ;
    nempty_row = 0 ;
    nnewlyempty_row = 0 ;

    /* === Kill empty columns =============================================== */

    /* Put the empty columns at the end in their natural order, so that LU */
    /* factorization can proceed as far as possible. */
    for (c = n_col-1 ; c >= 0 ; c--)
    {
	deg = Col [c].length ;
	if (deg == 0)
	{
	    /* this is a empty column, kill and order it last of its cset */
	    Col [c].shared2.order = --head [CMEMBER (c)] ;
	    --n_col2 ;
	    dead_cols [CMEMBER (c)] ++ ;
	    nempty_col++ ;
	    KILL_PRINCIPAL_COL (c) ;
	}
    }
    DEBUG1 (("ccolamd: null columns killed: "ID"\n", n_col - n_col2)) ;

    /* === Kill dense columns =============================================== */

    /* Put the dense columns at the end, in their natural order */
    for (c = n_col-1 ; c >= 0 ; c--)
    {
	/* skip any dead columns */
	if (COL_IS_DEAD (c))
	{
	    continue ;
	}
	deg = Col [c].length ;
	if (deg > dense_col_count)
	{
	    /* this is a dense column, kill and order it last of its cset */
	    Col [c].shared2.order = --head [CMEMBER (c)] ;
	    --n_col2 ;
	    dead_cols [CMEMBER (c)] ++ ;
	    ndense_col++ ;
	    /* decrement the row degrees */
	    cp = &A [Col [c].start] ;
	    cp_end = cp + Col [c].length ;
	    while (cp < cp_end)
	    {
		Row [*cp++].shared1.degree-- ;
	    }
	    KILL_PRINCIPAL_COL (c) ;
	}
    }
    DEBUG1 (("Dense and null columns killed: "ID"\n", n_col - n_col2)) ;

    /* === Kill dense and empty rows ======================================== */

    /* Note that there can now be empty rows, since dense columns have
     * been deleted.  These are "newly" empty rows. */

    ne = 0 ;
    for (r = 0 ; r < n_row ; r++)
    {
	deg = Row [r].shared1.degree ;
	ASSERT (deg >= 0 && deg <= n_col) ;
        if (deg > dense_row_count)
        {
            /* There is at least one dense row.  Continue ordering, but */
            /* symbolic factorization will be redone after ccolamd is done.*/
            ndense_row++ ;
        }
        if (deg == 0)
        {
            /* this is a newly empty row, or original empty row */
            ne++ ;
        }
	if (deg > dense_row_count || deg == 0)
	{
	    /* kill a dense or empty row */
	    KILL_ROW (r) ;
	    Row [r].thickness = 0 ;
	    --n_row2 ;
	}
	else
	{
	    /* keep track of max degree of remaining rows */
	    max_deg = MAX (max_deg, deg) ;
	}
    }
    nnewlyempty_row = ne - nempty_row ;
    DEBUG1 (("ccolamd: Dense and null rows killed: "ID"\n", n_row - n_row2)) ;

    /* === Compute initial column scores ==================================== */

    /* At this point the row degrees are accurate.  They reflect the number */
    /* of "live" (non-dense) columns in each row.  No empty rows exist. */
    /* Some "live" columns may contain only dead rows, however.  These are */
    /* pruned in the code below. */

    /* now find the initial COLMMD score for each column */
    for (c = n_col-1 ; c >= 0 ; c--)
    {
	/* skip dead column */
	if (COL_IS_DEAD (c))
	{
	    continue ;
	}
	score = 0 ;
	cp = &A [Col [c].start] ;
	new_cp = cp ;
	cp_end = cp + Col [c].length ;
	while (cp < cp_end)
	{
	    /* get a row */
	    row = *cp++ ;
	    /* skip if dead */
	    if (ROW_IS_DEAD (row))
	    {
		continue ;
	    }
	    /* compact the column */
	    *new_cp++ = row ;
	    /* add row's external degree */
	    score += Row [row].shared1.degree - 1 ;
	    /* guard against integer overflow */
	    score = MIN (score, n_col) ;
	}
	/* determine pruned column length */
	col_length = (Int) (new_cp - &A [Col [c].start]) ;
	if (col_length == 0)
	{
	    /* a newly-made null column (all rows in this col are "dense" */
	    /* and have already been killed) */
	    DEBUG1 (("Newly null killed: "ID"\n", c)) ;
	    Col [c].shared2.order = -- head [CMEMBER (c)] ;
	    --n_col2 ;
	    dead_cols [CMEMBER (c)] ++ ;
            nnewlyempty_col++ ;
	    KILL_PRINCIPAL_COL (c) ;
	}
	else
	{
	    /* set column length and set score */
	    ASSERT (score >= 0) ;
	    ASSERT (score <= n_col) ;
	    Col [c].length = col_length ;
	    Col [c].shared2.score = score ;
	}
    }
    DEBUG1 (("ccolamd: Dense, null, and newly-null columns killed: "ID"\n",
    	n_col-n_col2)) ;

    /* At this point, all empty rows and columns are dead.  All live columns */
    /* are "clean" (containing no dead rows) and simplicial (no supercolumns */
    /* yet).  Rows may contain dead columns, but all live rows contain at */
    /* least one live column. */

#ifndef NDEBUG
    debug_count = 0 ;
#endif

    /* clear the hash buckets */
    for (c = 0 ; c <= n_col ; c++)
    {
	head [c] = EMPTY ;
    }

#ifndef NDEBUG
    debug_structures (n_row, n_col, Row, Col, A, cmember, cset_start) ;
#endif

    /* === Return number of remaining columns, and max row degree =========== */

    *p_n_col2 = n_col2 ;
    *p_n_row2 = n_row2 ;
    *p_max_deg = max_deg ;
    *p_ndense_row = ndense_row ;
    *p_nempty_row = nempty_row ;        /* original empty rows */
    *p_nnewlyempty_row = nnewlyempty_row ;
    *p_ndense_col = ndense_col ;
    *p_nempty_col = nempty_col ;        /* original empty cols */
    *p_nnewlyempty_col = nnewlyempty_col ;
}


/* ========================================================================== */
/* === find_ordering ======================================================== */
/* ========================================================================== */

/*
 *   Order the principal columns of the supercolumn form of the matrix
 *  (no supercolumns on input).  Uses a minimum approximate column minimum
 *  degree ordering method.  Not user-callable.
 */

PRIVATE Int find_ordering	/* return the number of garbage collections */
(
    /* === Parameters ======================================================= */

    Int n_row,			/* number of rows of A */
    Int n_col,			/* number of columns of A */
    Int Alen,			/* size of A, 2*nnz + n_col or larger */
    CColamd_Row Row [ ],	/* of size n_row+1 */
    CColamd_Col Col [ ],	/* of size n_col+1 */
    Int A [ ],			/* column form and row form of A */
    Int head [ ],		/* of size n_col+1 */
#ifndef NDEBUG
    Int n_col2,			/* Remaining columns to order */
#endif
    Int max_deg,		/* Maximum row degree */
    Int pfree,			/* index of first free slot (2*nnz on entry) */
    Int cset [ ],		/* constraint set of A */
    Int cset_start [ ],		/* pointer to the start of every cset */
#ifndef NDEBUG
    Int n_cset,			/* number of csets */
#endif
    Int cmember [ ],		/* col -> cset mapping */
    Int Front_npivcol [ ],
    Int Front_nrows [ ],
    Int Front_ncols [ ],
    Int Front_parent [ ],
    Int Front_cols [ ],
    Int *p_nfr,                /* number of fronts */
    Int aggressive,
    Int InFront [ ],
    Int order_for_lu
)
{
    /* === Local variables ================================================== */

    Int k ;			/* current pivot ordering step */
    Int pivot_col ;		/* current pivot column */
    Int *cp ;			/* a column pointer */
    Int *rp ;			/* a row pointer */
    Int pivot_row ;		/* current pivot row */
    Int *new_cp ;		/* modified column pointer */
    Int *new_rp ;		/* modified row pointer */
    Int pivot_row_start ;	/* pointer to start of pivot row */
    Int pivot_row_degree ;	/* number of columns in pivot row */
    Int pivot_row_length ;	/* number of supercolumns in pivot row */
    Int pivot_col_score ;	/* score of pivot column */
    Int needed_memory ;		/* free space needed for pivot row */
    Int *cp_end ;		/* pointer to the end of a column */
    Int *rp_end ;		/* pointer to the end of a row */
    Int row ;			/* a row index */
    Int col ;			/* a column index */
    Int max_score ;		/* maximum possible score */
    Int cur_score ;		/* score of current column */
    unsigned Int hash ;		/* hash value for supernode detection */
    Int head_column ;		/* head of hash bucket */
    Int first_col ;		/* first column in hash bucket */
    Int tag_mark ;		/* marker value for mark array */
    Int row_mark ;		/* Row [row].shared2.mark */
    Int set_difference ;	/* set difference size of row with pivot row */
    Int min_score ;		/* smallest column score */
    Int col_thickness ;		/* "thickness" (no. of columns in a supercol) */
    Int max_mark ;		/* maximum value of tag_mark */
    Int pivot_col_thickness ;	/* number of columns represented by pivot col */
    Int prev_col ;		/* Used by Dlist operations. */
    Int next_col ;		/* Used by Dlist operations. */
    Int ngarbage ;		/* number of garbage collections performed */
    Int current_set ;		/* consraint set that is being ordered */
    Int score ;			/* score of a column */
    Int colstart ;		/* pointer to first column in current cset */
    Int colend ;		/* pointer to last column in current cset */
    Int deadcol ;		/* number of dense & null columns in a cset */

#ifndef NDEBUG
    Int debug_d ;		/* debug loop counter */
    Int debug_step = 0 ;	/* debug loop counter */
    Int cols_thickness = 0 ;	/* the thickness of the columns in current */
    				/* cset degreelist and in pivot row pattern. */
#endif

    Int pivot_row_thickness ;   /* number of rows represented by pivot row */
    Int nfr = 0 ;               /* number of fronts */
    Int child ;

    /* === Initialization and clear mark ==================================== */

    max_mark = Int_MAX - n_col ;	/* Int_MAX defined in <limits.h> */
    tag_mark = clear_mark (0, max_mark, n_row, Row) ;
    min_score = 0 ;
    ngarbage = 0 ;
    current_set = -1 ;
    deadcol = 0 ;
    DEBUG1 (("ccolamd: Ordering, n_col2="ID"\n", n_col2)) ;

    for (row = 0 ; row < n_row ; row++)
    {
        InFront [row] = EMPTY ;
    }

    /* === Order the columns ================================================ */

    for (k = 0 ; k < n_col ; /* 'k' is incremented below */)
    {

	/* make sure degree list isn't empty */
	ASSERT (min_score >= 0) ;
	ASSERT (min_score <= n_col) ;
	ASSERT (head [min_score] >= EMPTY) ;

#ifndef NDEBUG
	for (debug_d = 0 ; debug_d < min_score ; debug_d++)
	{
	    ASSERT (head [debug_d] == EMPTY) ;
	}
#endif

	/* Initialize the degree list with columns from next non-empty cset */

	while ((k+deadcol) == cset_start [current_set+1])
	{
	    current_set++ ;
	    DEBUG1 (("\n\n\n============ CSET: "ID"\n", current_set)) ;
	    k += deadcol ;	/* jump to start of next cset */
  	    deadcol = 0 ;	/* reset dead column count */

	    ASSERT ((current_set == n_cset) == (k == n_col)) ;

	    /* return if all columns are ordered. */
	    if (k == n_col)
	    {
		*p_nfr = nfr ;
	    	return (ngarbage) ;
	    }

#ifndef NDEBUG
	    for (col = 0 ; col <= n_col ; col++)
	    {
	        ASSERT (head [col] == EMPTY) ;
	    }
#endif

	    min_score = n_col ;
	    colstart = cset_start [current_set] ;
	    colend = cset_start [current_set+1] ;

	    while (colstart < colend)
	    {
		col = cset [colstart++] ;

		if (COL_IS_DEAD(col))
		{
		    DEBUG1 (("Column "ID" is dead\n", col)) ;
		    /* count dense and null columns */
		    if (Col [col].shared2.order != EMPTY)
		    {
			deadcol++ ;
		    }
		    continue ;
		}

		/* only add principal columns in current set to degree lists */
		ASSERT (CMEMBER (col) == current_set) ;

		score = Col [col].shared2.score ;
		DEBUG1 (("Column "ID" is alive, score "ID"\n", col, score)) ;

		ASSERT (min_score >= 0) ;
		ASSERT (min_score <= n_col) ;
		ASSERT (score >= 0) ;
		ASSERT (score <= n_col) ;
		ASSERT (head [score] >= EMPTY) ;

		/* now add this column to dList at proper score location */
		next_col = head [score] ;
		Col [col].shared3.prev = EMPTY ;
		Col [col].shared4.degree_next = next_col ;

		/* if there already was a column with the same score, set its */
		/* previous pointer to this new column */
		if (next_col != EMPTY)
		{
		    Col [next_col].shared3.prev = col ;
		}
		head [score] = col ;

		/* see if this score is less than current min */
		min_score = MIN (min_score, score) ;
	    }

#ifndef NDEBUG
	    DEBUG1 (("degree lists initialized \n")) ;
	    debug_deg_lists (n_row, n_col, Row, Col, head, min_score,
		((cset_start [current_set+1]-cset_start [current_set])-deadcol),
		max_deg) ;
#endif
	}

#ifndef NDEBUG
	if (debug_step % 100 == 0)
	{
	    DEBUG2 (("\n...   Step k: "ID" out of n_col2: "ID"\n", k, n_col2)) ;
	}
	else
	{
	    DEBUG3 (("\n------Step k: "ID" out of n_col2: "ID"\n", k, n_col2)) ;
	}
	debug_step++ ;
	DEBUG1 (("start of step k="ID": ", k)) ;
	debug_deg_lists (n_row, n_col, Row, Col, head,
	     min_score, cset_start [current_set+1]-(k+deadcol), max_deg) ;
	debug_matrix (n_row, n_col, Row, Col, A) ;
#endif

	/* === Select pivot column, and order it ============================ */

	while (head [min_score] == EMPTY && min_score < n_col)
	{
	    min_score++ ;
	}

	pivot_col = head [min_score] ;

	ASSERT (pivot_col >= 0 && pivot_col <= n_col) ;
	next_col = Col [pivot_col].shared4.degree_next ;
	head [min_score] = next_col ;
	if (next_col != EMPTY)
	{
	    Col [next_col].shared3.prev = EMPTY ;
	}

	ASSERT (COL_IS_ALIVE (pivot_col)) ;

	/* remember score for defrag check */
	pivot_col_score = Col [pivot_col].shared2.score ;

	/* the pivot column is the kth column in the pivot order */
	Col [pivot_col].shared2.order = k ;

	/* increment order count by column thickness */
	pivot_col_thickness = Col [pivot_col].shared1.thickness ;
	k += pivot_col_thickness ;
	ASSERT (pivot_col_thickness > 0) ;
	DEBUG3 (("Pivot col: "ID" thick "ID"\n", pivot_col,
		    pivot_col_thickness)) ;

	/* === Garbage_collection, if necessary ============================= */

	needed_memory = MIN (pivot_col_score, n_col - k) ;
	if (pfree + needed_memory >= Alen)
	{
	    pfree = garbage_collection (n_row, n_col, Row, Col, A, &A [pfree]) ;
	    ngarbage++ ;
	    /* after garbage collection we will have enough */
	    ASSERT (pfree + needed_memory < Alen) ;
	    /* garbage collection has wiped out Row [ ].shared2.mark array */
	    tag_mark = clear_mark (0, max_mark, n_row, Row) ;

#ifndef NDEBUG
	    debug_matrix (n_row, n_col, Row, Col, A) ;
#endif
	}

	/* === Compute pivot row pattern ==================================== */

	/* get starting location for this new merged row */
	pivot_row_start = pfree ;

	/* initialize new row counts to zero */
	pivot_row_degree = 0 ;
        pivot_row_thickness = 0 ;

	/* tag pivot column as having been visited so it isn't included */
	/* in merged pivot row */
	Col [pivot_col].shared1.thickness = -pivot_col_thickness ;

	/* pivot row is the union of all rows in the pivot column pattern */
	cp = &A [Col [pivot_col].start] ;
	cp_end = cp + Col [pivot_col].length ;
	while (cp < cp_end)
	{
	    /* get a row */
	    row = *cp++ ;
	    ASSERT (row >= 0 && row < n_row) ;
	    DEBUG4 (("Pivcol pattern "ID" "ID"\n", ROW_IS_ALIVE (row), row)) ;
	    /* skip if row is dead */
	    if (ROW_IS_ALIVE (row))
	    {
		/* sum the thicknesses of all the rows */
		pivot_row_thickness += Row [row].thickness ;

		rp = &A [Row [row].start] ;
		rp_end = rp + Row [row].length ;
		while (rp < rp_end)
		{
		    /* get a column */
		    col = *rp++ ;
		    /* add the column, if alive and untagged */
		    col_thickness = Col [col].shared1.thickness ;
		    if (col_thickness > 0 && COL_IS_ALIVE (col))
		    {
			/* tag column in pivot row */
			Col [col].shared1.thickness = -col_thickness ;
			ASSERT (pfree < Alen) ;
			/* place column in pivot row */
			A [pfree++] = col ;
			pivot_row_degree += col_thickness ;
			DEBUG4 (("\t\t\tNew live col in pivrow: "ID"\n",col)) ;
		    }
#ifndef NDEBUG
		    if (col_thickness < 0 && COL_IS_ALIVE (col))
		    {
			DEBUG4 (("\t\t\tOld live col in pivrow: "ID"\n",col)) ;
		    }
#endif
		}
	    }
	}

        /* pivot_row_thickness is the number of rows in frontal matrix */
        /* including both pivotal rows and nonpivotal rows */

	/* clear tag on pivot column */
	Col [pivot_col].shared1.thickness = pivot_col_thickness ;
	max_deg = MAX (max_deg, pivot_row_degree) ;

#ifndef NDEBUG
	DEBUG3 (("check2\n")) ;
	debug_mark (n_row, Row, tag_mark, max_mark) ;
#endif

	/* === Kill all rows used to construct pivot row ==================== */

	/* also kill pivot row, temporarily */
	cp = &A [Col [pivot_col].start] ;
	cp_end = cp + Col [pivot_col].length ;
	while (cp < cp_end)
	{
	    /* may be killing an already dead row */
	    row = *cp++ ;
	    DEBUG3 (("Kill row in pivot col: "ID"\n", row)) ;
	    ASSERT (row >= 0 && row < n_row) ;
            if (ROW_IS_ALIVE (row))
            {
                if (Row [row].front != EMPTY)
                {
                    /* This row represents a frontal matrix. */
                    /* Row [row].front is a child of current front */
                    child = Row [row].front ;
                    Front_parent [child] = nfr ;
                    DEBUG1 (("Front "ID" => front "ID", normal\n", child, nfr));
                }
                else
                {
                    /* This is an original row.  Keep track of which front
                     * is its parent in the row-merge tree. */
                    InFront [row] = nfr ;
                    DEBUG1 (("Row "ID" => front "ID", normal\n", row, nfr)) ;
                }
            }

            KILL_ROW (row) ;
            Row [row].thickness = 0 ;
	}

	/* === Select a row index to use as the new pivot row =============== */

	pivot_row_length = pfree - pivot_row_start ;
	if (pivot_row_length > 0)
	{
	    /* pick the "pivot" row arbitrarily (first row in col) */
	    pivot_row = A [Col [pivot_col].start] ;
	    DEBUG3 (("Pivotal row is "ID"\n", pivot_row)) ;
	}
	else
	{
	    /* there is no pivot row, since it is of zero length */
	    pivot_row = EMPTY ;
	    ASSERT (pivot_row_length == 0) ;
	}
	ASSERT (Col [pivot_col].length > 0 || pivot_row_length == 0) ;

	/* === Approximate degree computation =============================== */

	/* Here begins the computation of the approximate degree.  The column */
	/* score is the sum of the pivot row "length", plus the size of the */
	/* set differences of each row in the column minus the pattern of the */
	/* pivot row itself.  The column ("thickness") itself is also */
	/* excluded from the column score (we thus use an approximate */
	/* external degree). */

	/* The time taken by the following code (compute set differences, and */
	/* add them up) is proportional to the size of the data structure */
	/* being scanned - that is, the sum of the sizes of each column in */
	/* the pivot row.  Thus, the amortized time to compute a column score */
	/* is proportional to the size of that column (where size, in this */
	/* context, is the column "length", or the number of row indices */
	/* in that column).  The number of row indices in a column is */
	/* monotonically non-decreasing, from the length of the original */
	/* column on input to colamd. */

	/* === Compute set differences ====================================== */

	DEBUG3 (("** Computing set differences phase. **\n")) ;

	/* pivot row is currently dead - it will be revived later. */

	DEBUG3 (("Pivot row: ")) ;
	/* for each column in pivot row */
	rp = &A [pivot_row_start] ;
	rp_end = rp + pivot_row_length ;
	while (rp < rp_end)
	{
	    col = *rp++ ;
	    ASSERT (COL_IS_ALIVE (col) && col != pivot_col) ;
	    DEBUG3 (("Col: "ID"\n", col)) ;

	    /* clear tags used to construct pivot row pattern */
	    col_thickness = -Col [col].shared1.thickness ;
	    ASSERT (col_thickness > 0) ;
	    Col [col].shared1.thickness = col_thickness ;

	    /* === Remove column from degree list =========================== */

	    /* only columns in current_set will be in degree list */
	    if (CMEMBER (col) == current_set)
	    {
#ifndef NDEBUG
		cols_thickness += col_thickness ;
#endif
		cur_score = Col [col].shared2.score ;
		prev_col = Col [col].shared3.prev ;
		next_col = Col [col].shared4.degree_next ;
		DEBUG3 (("        cur_score "ID" prev_col "ID" next_col "ID"\n",
			cur_score, prev_col, next_col)) ;
		ASSERT (cur_score >= 0) ;
		ASSERT (cur_score <= n_col) ;
		ASSERT (cur_score >= EMPTY) ;
		if (prev_col == EMPTY)
		{
		    head [cur_score] = next_col ;
		}
		else
		{
		    Col [prev_col].shared4.degree_next = next_col ;
		}
		if (next_col != EMPTY)
		{
		    Col [next_col].shared3.prev = prev_col ;
		}
	    }

	    /* === Scan the column ========================================== */

	    cp = &A [Col [col].start] ;
	    cp_end = cp + Col [col].length ;
	    while (cp < cp_end)
	    {
		/* get a row */
		row = *cp++ ;
		row_mark = Row [row].shared2.mark ;
		/* skip if dead */
		if (ROW_IS_MARKED_DEAD (row_mark))
		{
		    continue ;
		}
		ASSERT (row != pivot_row) ;
		set_difference = row_mark - tag_mark ;
		/* check if the row has been seen yet */
		if (set_difference < 0)
		{
		    ASSERT (Row [row].shared1.degree <= max_deg) ;
		    set_difference = Row [row].shared1.degree ;
		}
		/* subtract column thickness from this row's set difference */
		set_difference -= col_thickness ;
		ASSERT (set_difference >= 0) ;
		/* absorb this row if the set difference becomes zero */
		if (set_difference == 0 && aggressive)
		{
		    DEBUG3 (("aggressive absorption. Row: "ID"\n", row)) ;

                    if (Row [row].front != EMPTY)
                    {
                        /* Row [row].front is a child of current front. */
                        child = Row [row].front ;
                        Front_parent [child] = nfr ;
                        DEBUG1 (("Front "ID" => front "ID", aggressive\n",
                                    child, nfr)) ;
                    }
                    else
                    {
                        /* this is an original row.  Keep track of which front
                         * assembles it, for the row-merge tree */
                        InFront [row] = nfr ;
                        DEBUG1 (("Row "ID" => front "ID", aggressive\n",
                                    row, nfr)) ;
                    }

                    KILL_ROW (row) ;

                    /* sum the thicknesses of all the rows */
                    pivot_row_thickness += Row [row].thickness ;
                    Row [row].thickness = 0 ;
		}
		else
		{
		    /* save the new mark */
		    Row [row].shared2.mark = set_difference + tag_mark ;
		}
	    }
	}

#ifndef NDEBUG
	debug_deg_lists (n_row, n_col, Row, Col, head, min_score,
	cset_start [current_set+1]-(k+deadcol)-(cols_thickness),
		max_deg) ;
	cols_thickness = 0 ;
#endif

	/* === Add up set differences for each column ======================= */

	DEBUG3 (("** Adding set differences phase. **\n")) ;

	/* for each column in pivot row */
	rp = &A [pivot_row_start] ;
	rp_end = rp + pivot_row_length ;
	while (rp < rp_end)
	{
	    /* get a column */
	    col = *rp++ ;
	    ASSERT (COL_IS_ALIVE (col) && col != pivot_col) ;
	    hash = 0 ;
	    cur_score = 0 ;
	    cp = &A [Col [col].start] ;
	    /* compact the column */
	    new_cp = cp ;
	    cp_end = cp + Col [col].length ;

	    DEBUG4 (("Adding set diffs for Col: "ID".\n", col)) ;

	    while (cp < cp_end)
	    {
		/* get a row */
		row = *cp++ ;
		ASSERT (row >= 0 && row < n_row) ;
		row_mark = Row [row].shared2.mark ;
		/* skip if dead */
		if (ROW_IS_MARKED_DEAD (row_mark))
		{
		    DEBUG4 ((" Row "ID", dead\n", row)) ;
		    continue ;
		}
		DEBUG4 ((" Row "ID", set diff "ID"\n", row, row_mark-tag_mark));
                ASSERT (row_mark >= tag_mark) ;
		/* compact the column */
		*new_cp++ = row ;
		/* compute hash function */
		hash += row ;
		/* add set difference */
		cur_score += row_mark - tag_mark ;
		/* integer overflow... */
		cur_score = MIN (cur_score, n_col) ;
	    }

	    /* recompute the column's length */
	    Col [col].length = (Int) (new_cp - &A [Col [col].start]) ;

	    /* === Further mass elimination ================================= */

	    if (Col [col].length == 0 && CMEMBER (col) == current_set)
	    {
		DEBUG4 (("further mass elimination. Col: "ID"\n", col)) ;
		/* nothing left but the pivot row in this column */
		KILL_PRINCIPAL_COL (col) ;
		pivot_row_degree -= Col [col].shared1.thickness ;
		ASSERT (pivot_row_degree >= 0) ;
		/* order it */
		Col [col].shared2.order = k ;
		/* increment order count by column thickness */
		k += Col [col].shared1.thickness ;
                pivot_col_thickness += Col [col].shared1.thickness ;
                /* add to column list of front */
#ifndef NDEBUG
                DEBUG1 (("Mass")) ;
                dump_super (col, Col, n_col) ;
#endif
                Col [Col [col].lastcol].nextcol = Front_cols [nfr] ;
                Front_cols [nfr] = col ;
	    }
	    else
	    {
		/* === Prepare for supercolumn detection ==================== */

		DEBUG4 (("Preparing supercol detection for Col: "ID".\n", col));

		/* save score so far */
		Col [col].shared2.score = cur_score ;

		/* add column to hash table, for supercolumn detection */
		hash %= n_col + 1 ;

		DEBUG4 ((" Hash = "ID", n_col = "ID".\n", hash, n_col)) ;
		ASSERT (((Int) hash) <= n_col) ;

		head_column = head [hash] ;
		if (head_column > EMPTY)
		{
		    /* degree list "hash" is non-empty, use prev (shared3) of */
		    /* first column in degree list as head of hash bucket */
		    first_col = Col [head_column].shared3.headhash ;
		    Col [head_column].shared3.headhash = col ;
		}
		else
		{
		    /* degree list "hash" is empty, use head as hash bucket */
		    first_col = - (head_column + 2) ;
		    head [hash] = - (col + 2) ;
		}
		Col [col].shared4.hash_next = first_col ;

		/* save hash function in Col [col].shared3.hash */
		Col [col].shared3.hash = (Int) hash ;
		ASSERT (COL_IS_ALIVE (col)) ;
	    }
	}

	/* The approximate external column degree is now computed.  */

	/* === Supercolumn detection ======================================== */

	DEBUG3 (("** Supercolumn detection phase. **\n")) ;

	detect_super_cols (
#ifndef NDEBUG
		n_col, Row,
#endif
		Col, A, head, pivot_row_start, pivot_row_length, cmember) ;

	/* === Kill the pivotal column ====================================== */

	DEBUG1 ((" KILLING column detect supercols "ID" \n", pivot_col)) ;
	KILL_PRINCIPAL_COL (pivot_col) ;

	/* add columns to column list of front */
#ifndef NDEBUG
	DEBUG1 (("Pivot")) ;
	dump_super (pivot_col, Col, n_col) ;
#endif
	Col [Col [pivot_col].lastcol].nextcol = Front_cols [nfr] ;
	Front_cols [nfr] = pivot_col ;

	/* === Clear mark =================================================== */

	tag_mark = clear_mark (tag_mark+max_deg+1, max_mark, n_row, Row) ;

#ifndef NDEBUG
	DEBUG3 (("check3\n")) ;
	debug_mark (n_row, Row, tag_mark, max_mark) ;
#endif

	/* === Finalize the new pivot row, and column scores ================ */

	DEBUG3 (("** Finalize scores phase. **\n")) ;

	/* for each column in pivot row */
	rp = &A [pivot_row_start] ;
	/* compact the pivot row */
	new_rp = rp ;
	rp_end = rp + pivot_row_length ;
	while (rp < rp_end)
	{
	    col = *rp++ ;
	    /* skip dead columns */
	    if (COL_IS_DEAD (col))
	    {
		continue ;
	    }
	    *new_rp++ = col ;
	    /* add new pivot row to column */
	    A [Col [col].start + (Col [col].length++)] = pivot_row ;

	    /* retrieve score so far and add on pivot row's degree. */
	    /* (we wait until here for this in case the pivot */
	    /* row's degree was reduced due to mass elimination). */
	    cur_score = Col [col].shared2.score + pivot_row_degree ;

	    /* calculate the max possible score as the number of */
	    /* external columns minus the 'k' value minus the */
	    /* columns thickness */
	    max_score = n_col - k - Col [col].shared1.thickness ;

	    /* make the score the external degree of the union-of-rows */
	    cur_score -= Col [col].shared1.thickness ;

	    /* make sure score is less or equal than the max score */
	    cur_score = MIN (cur_score, max_score) ;
	    ASSERT (cur_score >= 0) ;

	    /* store updated score */
	    Col [col].shared2.score = cur_score ;

	    /* === Place column back in degree list ========================= */

	    if (CMEMBER (col) == current_set)
	    {
		ASSERT (min_score >= 0) ;
		ASSERT (min_score <= n_col) ;
		ASSERT (cur_score >= 0) ;
		ASSERT (cur_score <= n_col) ;
		ASSERT (head [cur_score] >= EMPTY) ;
		next_col = head [cur_score] ;
		Col [col].shared4.degree_next = next_col ;
		Col [col].shared3.prev = EMPTY ;
		if (next_col != EMPTY)
		{
		    Col [next_col].shared3.prev = col ;
		}
		head [cur_score] = col ;
		/* see if this score is less than current min */
		min_score = MIN (min_score, cur_score) ;
	    }
	    else
	    {
		Col [col].shared4.degree_next = EMPTY ;
		Col [col].shared3.prev = EMPTY ;
	    }
	}

#ifndef NDEBUG
	debug_deg_lists (n_row, n_col, Row, Col, head,
		min_score, cset_start [current_set+1]-(k+deadcol), max_deg) ;
#endif

	/* frontal matrix can have more pivot cols than pivot rows for */
	/* singular matrices. */

	/* number of candidate pivot columns */
	Front_npivcol [nfr] = pivot_col_thickness ;

	/* all rows (not just size of contrib. block) */
	Front_nrows [nfr] = pivot_row_thickness ;

	/* all cols */
	Front_ncols [nfr] = pivot_col_thickness + pivot_row_degree ;

	Front_parent [nfr] = EMPTY ;

	pivot_row_thickness -= pivot_col_thickness ;
	DEBUG1 (("Front "ID" Pivot_row_thickness after pivot cols elim: "ID"\n",
	     nfr, pivot_row_thickness)) ;
	pivot_row_thickness = MAX (0, pivot_row_thickness) ;

	/* === Resurrect the new pivot row ================================== */

	if ((pivot_row_degree > 0 && pivot_row_thickness > 0 && (order_for_lu))
	   || (pivot_row_degree > 0 && (!order_for_lu)))
	{
	    /* update pivot row length to reflect any cols that were killed */
	    /* during super-col detection and mass elimination */
	    Row [pivot_row].start  = pivot_row_start ;
	    Row [pivot_row].length = (Int) (new_rp - &A[pivot_row_start]) ;
	    Row [pivot_row].shared1.degree = pivot_row_degree ;
	    Row [pivot_row].shared2.mark = 0 ;
	    Row [pivot_row].thickness = pivot_row_thickness ;
	    Row [pivot_row].front = nfr ;
	    /* pivot row is no longer dead */
	    DEBUG1 (("Resurrect Pivot_row "ID" deg: "ID"\n",
			pivot_row, pivot_row_degree)) ;
	}

#ifndef NDEBUG
	DEBUG1 (("Front "ID" : "ID" "ID" "ID" ", nfr,
		 Front_npivcol [nfr], Front_nrows [nfr], Front_ncols [nfr])) ;
	DEBUG1 ((" cols:[ ")) ;
	debug_d = 0 ;
	for (col = Front_cols [nfr] ; col != EMPTY ; col = Col [col].nextcol)
	{
		DEBUG1 ((" "ID, col)) ;
		ASSERT (col >= 0 && col < n_col) ;
		ASSERT (COL_IS_DEAD (col)) ;
		debug_d++ ;
		ASSERT (debug_d <= pivot_col_thickness) ;
	}
	ASSERT (debug_d == pivot_col_thickness) ;
	DEBUG1 ((" ]\n ")) ;
#endif
	 nfr++ ; /* one more front */
    }

    /* === All principal columns have now been ordered ====================== */

    *p_nfr = nfr ;
    return (ngarbage) ;
}


/* ========================================================================== */
/* === detect_super_cols ==================================================== */
/* ========================================================================== */

/*
 *  Detects supercolumns by finding matches between columns in the hash buckets.
 *  Check amongst columns in the set A [row_start ... row_start + row_length-1].
 *  The columns under consideration are currently *not* in the degree lists,
 *  and have already been placed in the hash buckets.
 *
 *  The hash bucket for columns whose hash function is equal to h is stored
 *  as follows:
 *
 *	if head [h] is >= 0, then head [h] contains a degree list, so:
 *
 *		head [h] is the first column in degree bucket h.
 *		Col [head [h]].headhash gives the first column in hash bucket h.
 *
 *	otherwise, the degree list is empty, and:
 *
 *		-(head [h] + 2) is the first column in hash bucket h.
 *
 *  For a column c in a hash bucket, Col [c].shared3.prev is NOT a "previous
 *  column" pointer.  Col [c].shared3.hash is used instead as the hash number
 *  for that column.  The value of Col [c].shared4.hash_next is the next column
 *  in the same hash bucket.
 *
 *  Assuming no, or "few" hash collisions, the time taken by this routine is
 *  linear in the sum of the sizes (lengths) of each column whose score has
 *  just been computed in the approximate degree computation.
 *  Not user-callable.
 */

PRIVATE void detect_super_cols
(
    /* === Parameters ======================================================= */

#ifndef NDEBUG
    /* these two parameters are only needed when debugging is enabled: */
    Int n_col,			/* number of columns of A */
    CColamd_Row Row [ ],	/* of size n_row+1 */
#endif

    CColamd_Col Col [ ],	/* of size n_col+1 */
    Int A [ ],			/* row indices of A */
    Int head [ ],		/* head of degree lists and hash buckets */
    Int row_start,		/* pointer to set of columns to check */
    Int row_length,		/* number of columns to check */
    Int cmember [ ]		/* col -> cset mapping */
)
{
    /* === Local variables ================================================== */

    Int hash ;			/* hash value for a column */
    Int *rp ;			/* pointer to a row */
    Int c ;			/* a column index */
    Int super_c ;		/* column index of the column to absorb into */
    Int *cp1 ;			/* column pointer for column super_c */
    Int *cp2 ;			/* column pointer for column c */
    Int length ;		/* length of column super_c */
    Int prev_c ;		/* column preceding c in hash bucket */
    Int i ;			/* loop counter */
    Int *rp_end ;		/* pointer to the end of the row */
    Int col ;			/* a column index in the row to check */
    Int head_column ;		/* first column in hash bucket or degree list */
    Int first_col ;		/* first column in hash bucket */

    /* === Consider each column in the row ================================== */

    rp = &A [row_start] ;
    rp_end = rp + row_length ;
    while (rp < rp_end)
    {
	col = *rp++ ;
	if (COL_IS_DEAD (col))
	{
	    continue ;
	}

	/* get hash number for this column */
	hash = Col [col].shared3.hash ;
	ASSERT (hash <= n_col) ;

	/* === Get the first column in this hash bucket ===================== */

	head_column = head [hash] ;
	if (head_column > EMPTY)
	{
	    first_col = Col [head_column].shared3.headhash ;
	}
	else
	{
	    first_col = - (head_column + 2) ;
	}

	/* === Consider each column in the hash bucket ====================== */

	for (super_c = first_col ; super_c != EMPTY ;
	    super_c = Col [super_c].shared4.hash_next)
	{
	    ASSERT (COL_IS_ALIVE (super_c)) ;
	    ASSERT (Col [super_c].shared3.hash == hash) ;
	    length = Col [super_c].length ;

	    /* prev_c is the column preceding column c in the hash bucket */
	    prev_c = super_c ;

	    /* === Compare super_c with all columns after it ================ */

	    for (c = Col [super_c].shared4.hash_next ;
		 c != EMPTY ; c = Col [c].shared4.hash_next)
	    {
		ASSERT (c != super_c) ;
		ASSERT (COL_IS_ALIVE (c)) ;
		ASSERT (Col [c].shared3.hash == hash) ;

		/* not identical if lengths or scores are different, */
		/* or if in different constraint sets */
		if (Col [c].length != length ||
		    Col [c].shared2.score != Col [super_c].shared2.score
		    || CMEMBER (c) != CMEMBER (super_c))
		{
		    prev_c = c ;
		    continue ;
		}

		/* compare the two columns */
		cp1 = &A [Col [super_c].start] ;
		cp2 = &A [Col [c].start] ;

		for (i = 0 ; i < length ; i++)
		{
		    /* the columns are "clean" (no dead rows) */
		    ASSERT (ROW_IS_ALIVE (*cp1)) ;
		    ASSERT (ROW_IS_ALIVE (*cp2)) ;
		    /* row indices will same order for both supercols, */
		    /* no gather scatter nessasary */
		    if (*cp1++ != *cp2++)
		    {
			break ;
		    }
		}

		/* the two columns are different if the for-loop "broke" */
	        /* super columns should belong to the same constraint set */
		if (i != length)
		{
		    prev_c = c ;
		    continue ;
		}

		/* === Got it!  two columns are identical =================== */

		ASSERT (Col [c].shared2.score == Col [super_c].shared2.score) ;

		Col [super_c].shared1.thickness += Col [c].shared1.thickness ;
		Col [c].shared1.parent = super_c ;
		KILL_NON_PRINCIPAL_COL (c) ;
		/* order c later, in order_children() */
		Col [c].shared2.order = EMPTY ;
		/* remove c from hash bucket */
		Col [prev_c].shared4.hash_next = Col [c].shared4.hash_next ;

		/* add c to end of list of super_c */
		ASSERT (Col [super_c].lastcol >= 0) ;
		ASSERT (Col [super_c].lastcol < n_col) ;
		Col [Col [super_c].lastcol].nextcol = c ;
		Col [super_c].lastcol = Col [c].lastcol ;
#ifndef NDEBUG
		/* dump the supercolumn */
		DEBUG1 (("Super")) ;
		dump_super (super_c, Col, n_col) ;
#endif
	    }
	}

	/* === Empty this hash bucket ======================================= */

	if (head_column > EMPTY)
	{
	    /* corresponding degree list "hash" is not empty */
	    Col [head_column].shared3.headhash = EMPTY ;
	}
	else
	{
	    /* corresponding degree list "hash" is empty */
	    head [hash] = EMPTY ;
	}
    }
}


/* ========================================================================== */
/* === garbage_collection =================================================== */
/* ========================================================================== */

/*
 *  Defragments and compacts columns and rows in the workspace A.  Used when
 *  all avaliable memory has been used while performing row merging.  Returns
 *  the index of the first free position in A, after garbage collection.  The
 *  time taken by this routine is linear is the size of the array A, which is
 *  itself linear in the number of nonzeros in the input matrix.
 *  Not user-callable.
 */

PRIVATE Int garbage_collection  /* returns the new value of pfree */
(
    /* === Parameters ======================================================= */

    Int n_row,			/* number of rows */
    Int n_col,			/* number of columns */
    CColamd_Row Row [ ],	/* row info */
    CColamd_Col Col [ ],	/* column info */
    Int A [ ],			/* A [0 ... Alen-1] holds the matrix */
    Int *pfree			/* &A [0] ... pfree is in use */
)
{
    /* === Local variables ================================================== */

    Int *psrc ;			/* source pointer */
    Int *pdest ;		/* destination pointer */
    Int j ;			/* counter */
    Int r ;			/* a row index */
    Int c ;			/* a column index */
    Int length ;		/* length of a row or column */

#ifndef NDEBUG
    Int debug_rows ;
    DEBUG2 (("Defrag..\n")) ;
    for (psrc = &A[0] ; psrc < pfree ; psrc++) ASSERT (*psrc >= 0) ;
    debug_rows = 0 ;
#endif

    /* === Defragment the columns =========================================== */

    pdest = &A[0] ;
    for (c = 0 ; c < n_col ; c++)
    {
	if (COL_IS_ALIVE (c))
	{
	    psrc = &A [Col [c].start] ;

	    /* move and compact the column */
	    ASSERT (pdest <= psrc) ;
	    Col [c].start = (Int) (pdest - &A [0]) ;
	    length = Col [c].length ;
	    for (j = 0 ; j < length ; j++)
	    {
		r = *psrc++ ;
		if (ROW_IS_ALIVE (r))
		{
		    *pdest++ = r ;
		}
	    }
	    Col [c].length = (Int) (pdest - &A [Col [c].start]) ;
	}
    }

    /* === Prepare to defragment the rows =================================== */

    for (r = 0 ; r < n_row ; r++)
    {
	if (ROW_IS_DEAD (r) || (Row [r].length == 0))
	{
	    /* This row is already dead, or is of zero length.  Cannot compact
	     * a row of zero length, so kill it.  NOTE: in the current version,
	     * there are no zero-length live rows.  Kill the row (for the first
	     * time, or again) just to be safe. */
	    KILL_ROW (r) ;
	}
	else
	{
	    /* save first column index in Row [r].shared2.first_column */
	    psrc = &A [Row [r].start] ;
	    Row [r].shared2.first_column = *psrc ;
	    ASSERT (ROW_IS_ALIVE (r)) ;
	    /* flag the start of the row with the one's complement of row */
	    *psrc = ONES_COMPLEMENT (r) ;
#ifndef NDEBUG
	    debug_rows++ ;
#endif
	}
    }

    /* === Defragment the rows ============================================== */

    psrc = pdest ;
    while (psrc < pfree)
    {
	/* find a negative number ... the start of a row */
	if (*psrc++ < 0)
	{
	    psrc-- ;
	    /* get the row index */
	    r = ONES_COMPLEMENT (*psrc) ;
	    ASSERT (r >= 0 && r < n_row) ;
	    /* restore first column index */
	    *psrc = Row [r].shared2.first_column ;
	    ASSERT (ROW_IS_ALIVE (r)) ;

	    /* move and compact the row */
	    ASSERT (pdest <= psrc) ;
	    Row [r].start = (Int) (pdest - &A [0]) ;
	    length = Row [r].length ;
	    for (j = 0 ; j < length ; j++)
	    {
		c = *psrc++ ;
		if (COL_IS_ALIVE (c))
		{
		    *pdest++ = c ;
		}
	    }
	    Row [r].length = (Int) (pdest - &A [Row [r].start]) ;
#ifndef NDEBUG
	    debug_rows-- ;
#endif
	}
    }

    /* ensure we found all the rows */
    ASSERT (debug_rows == 0) ;

    /* === Return the new value of pfree ==================================== */

    return ((Int) (pdest - &A [0])) ;
}


/* ========================================================================== */
/* === clear_mark =========================================================== */
/* ========================================================================== */

/*
 *  Clears the Row [ ].shared2.mark array, and returns the new tag_mark.
 *  Return value is the new tag_mark.  Not user-callable.
 */

PRIVATE Int clear_mark	/* return the new value for tag_mark */
(
    /* === Parameters ======================================================= */

    Int tag_mark,	/* new value of tag_mark */
    Int max_mark,	/* max allowed value of tag_mark */

    Int n_row,		/* number of rows in A */
    CColamd_Row Row [ ]	/* Row [0 ... n_row-1].shared2.mark is set to zero */
)
{
    /* === Local variables ================================================== */

    Int r ;

    if (tag_mark <= 0 || tag_mark >= max_mark)
    {
	for (r = 0 ; r < n_row ; r++)
	{
	    if (ROW_IS_ALIVE (r))
	    {
		Row [r].shared2.mark = 0 ;
	    }
	}
	tag_mark = 1 ;
    }

    return (tag_mark) ;
}


/* ========================================================================== */
/* === print_report ========================================================= */
/* ========================================================================== */

/* No printing occurs if NPRINT is defined at compile time. */

PRIVATE void print_report
(
    char *method,
    Int stats [CCOLAMD_STATS]
)
{

    Int i1, i2, i3 ;

    PRINTF (("\n%s version %d.%d, %s: ", method,
	    CCOLAMD_MAIN_VERSION, CCOLAMD_SUB_VERSION, CCOLAMD_DATE)) ;

    if (!stats)
    {
    	PRINTF (("No statistics available.\n")) ;
	return ;
    }

    i1 = stats [CCOLAMD_INFO1] ;
    i2 = stats [CCOLAMD_INFO2] ;
    i3 = stats [CCOLAMD_INFO3] ;

    if (stats [CCOLAMD_STATUS] >= 0)
    {
    	PRINTF(("OK.  ")) ;
    }
    else
    {
    	PRINTF(("ERROR.  ")) ;
    }

    switch (stats [CCOLAMD_STATUS])
    {

	case CCOLAMD_OK_BUT_JUMBLED:

	    PRINTF(("Matrix has unsorted or duplicate row indices.\n")) ;

	    PRINTF(("%s: duplicate or out-of-order row indices:    "ID"\n",
		    method, i3)) ;

	    PRINTF(("%s: last seen duplicate or out-of-order row:  "ID"\n",
		    method, INDEX (i2))) ;

	    PRINTF(("%s: last seen in column:                      "ID"",
		    method, INDEX (i1))) ;

	    /* no break - fall through to next case instead */

	case CCOLAMD_OK:

	    PRINTF(("\n")) ;

 	    PRINTF(("%s: number of dense or empty rows ignored:    "ID"\n",
		    method, stats [CCOLAMD_DENSE_ROW])) ;

	    PRINTF(("%s: number of dense or empty columns ignored: "ID"\n",
		    method, stats [CCOLAMD_DENSE_COL])) ;

	    PRINTF(("%s: number of garbage collections performed:  "ID"\n",
		    method, stats [CCOLAMD_DEFRAG_COUNT])) ;
	    break ;

	case CCOLAMD_ERROR_A_not_present:

	    PRINTF(("Array A (row indices of matrix) not present.\n")) ;
	    break ;

	case CCOLAMD_ERROR_p_not_present:

	    PRINTF(("Array p (column pointers for matrix) not present.\n")) ;
	    break ;

	case CCOLAMD_ERROR_nrow_negative:

	    PRINTF(("Invalid number of rows ("ID").\n", i1)) ;
	    break ;

	case CCOLAMD_ERROR_ncol_negative:

	    PRINTF(("Invalid number of columns ("ID").\n", i1)) ;
	    break ;

	case CCOLAMD_ERROR_nnz_negative:

	    PRINTF(("Invalid number of nonzero entries ("ID").\n", i1)) ;
	    break ;

	case CCOLAMD_ERROR_p0_nonzero:

	    PRINTF(("Invalid column pointer, p [0] = "ID", must be 0.\n", i1)) ;
	    break ;

	case CCOLAMD_ERROR_A_too_small:

	    PRINTF(("Array A too small.\n")) ;
	    PRINTF(("        Need Alen >= "ID", but given only Alen = "ID".\n",
		    i1, i2)) ;
	    break ;

	case CCOLAMD_ERROR_col_length_negative:

	    PRINTF(("Column "ID" has a negative number of entries ("ID").\n",
		    INDEX (i1), i2)) ;
	    break ;

	case CCOLAMD_ERROR_row_index_out_of_bounds:

	    PRINTF(("Row index (row "ID") out of bounds ("ID" to "ID") in"
		    "column "ID".\n", INDEX (i2), INDEX (0), INDEX (i3-1),
		    INDEX (i1))) ;
	    break ;

	case CCOLAMD_ERROR_out_of_memory:

	    PRINTF(("Out of memory.\n")) ;
	    break ;

	case CCOLAMD_ERROR_invalid_cmember:

	    PRINTF(("cmember invalid\n")) ;
	    break ;
    }
}


/* ========================================================================= */
/* === "Expert" routines =================================================== */
/* ========================================================================= */

/* The following routines are visible outside this routine, but are not meant
 * to be called by the user.  They are meant for a future version of UMFPACK,
 * to replace UMFPACK internal routines with a similar name.
 */


/* ========================================================================== */
/* === CCOLAMD_apply_order ================================================== */
/* ========================================================================== */

/*
 * Apply post-ordering of supernodal elimination tree.
 */

GLOBAL void CCOLAMD_apply_order
(
    Int Front [ ],	    /* of size nn on input, size nfr on output */
    const Int Order [ ],    /* Order [i] = k, i in the range 0..nn-1,
			     * and k in the range 0..nfr-1, means that node
			     * i is the kth node in the postordered tree. */
    Int Temp [ ],	    /* workspace of size nfr */
    Int nn,		    /* nodes are numbered in the range 0..nn-1 */
    Int nfr		    /* the number of nodes actually in use */
)
{
    Int i, k ;
    for (i = 0 ; i < nn ; i++)
    {
	k = Order [i] ;
	ASSERT (k >= EMPTY && k < nfr) ;
	if (k != EMPTY)
	{
	    Temp [k] = Front [i] ;
	}
    }

    for (k = 0 ; k < nfr ; k++)
    {
	Front [k] = Temp [k] ;
    }
}


/* ========================================================================== */
/* === CCOLAMD_fsize ======================================================== */
/* ========================================================================== */

/* Determine the largest frontal matrix size for each subtree. 
 * Only required to sort the children of each
 * node prior to postordering the column elimination tree. */

GLOBAL void CCOLAMD_fsize
(
    Int nn,
    Int Fsize [ ],
    Int Fnrows [ ],
    Int Fncols [ ],
    Int Parent [ ],
    Int Npiv [ ]
)
{
    double dr, dc ;
    Int j, parent, frsize, r, c ;

    for (j = 0 ; j < nn ; j++)
    {
	Fsize [j] = EMPTY ;
    }

    /* ---------------------------------------------------------------------- */
    /* find max front size for tree rooted at node j, for each front j */
    /* ---------------------------------------------------------------------- */

    DEBUG1 (("\n\n========================================FRONTS:\n")) ;
    for (j = 0 ; j < nn ; j++)
    {
	if (Npiv [j] > 0)
	{
	    /* this is a frontal matrix */
	    parent = Parent [j] ;
	    r = Fnrows [j] ;
	    c = Fncols [j] ;
	    /* avoid integer overflow */
	    dr = (double) r ;
	    dc = (double) c ;
	    frsize = (INT_OVERFLOW (dr * dc)) ?  Int_MAX : (r * c) ;
	    DEBUG1 ((""ID" : npiv "ID" size "ID" parent "ID" ",
		j, Npiv [j], frsize, parent)) ;
	    Fsize [j] = MAX (Fsize [j], frsize) ;
	    DEBUG1 (("Fsize [j = "ID"] = "ID"\n", j, Fsize [j])) ;
	    if (parent != EMPTY)
	    {
		/* find the maximum frontsize of self and children */
		ASSERT (Npiv [parent] > 0) ;
		ASSERT (parent > j) ;
		Fsize [parent] = MAX (Fsize [parent], Fsize [j]) ;
		DEBUG1 (("Fsize [parent = "ID"] = "ID"\n",
		    parent, Fsize [parent]));
	    }
	}
    }
    DEBUG1 (("fsize done\n")) ;
}


/* ========================================================================= */
/* === CCOLAMD_postorder =================================================== */
/* ========================================================================= */

/* Perform a postordering (via depth-first search) of an assembly tree. */

GLOBAL void CCOLAMD_postorder
(
    /* inputs, not modified on output: */
    Int nn,		/* nodes are in the range 0..nn-1 */
    Int Parent [ ],	/* Parent [j] is the parent of j, or EMPTY if root */
    Int Nv [ ],		/* Nv [j] > 0 number of pivots represented by node j,
			 * or zero if j is not a node. */
    Int Fsize [ ],	/* Fsize [j]: size of node j */

    /* output, not defined on input: */
    Int Order [ ],	/* output post-order */

    /* workspaces of size nn: */
    Int Child [ ],
    Int Sibling [ ],
    Int Stack [ ],
    Int Front_cols [ ],

    /* input, not modified on output: */
    Int cmember [ ]
)
{
    Int i, j, k, parent, frsize, f, fprev, maxfrsize, bigfprev, bigf, fnext ;

    for (j = 0 ; j < nn ; j++)
    {
	Child [j] = EMPTY ;
	Sibling [j] = EMPTY ;
    }

    /* --------------------------------------------------------------------- */
    /* place the children in link lists - bigger elements tend to be last */
    /* --------------------------------------------------------------------- */

    for (j = nn-1 ; j >= 0 ; j--)
    {
	if (Nv [j] > 0)
	{
	    /* this is an element */
	    parent = Parent [j] ;
	    if (parent != EMPTY)
	    {
		/* place the element in link list of the children its parent */
		/* bigger elements will tend to be at the end of the list */
		Sibling [j] = Child [parent] ;
		if (CMEMBER (Front_cols[parent]) == CMEMBER (Front_cols[j]))
		{
		    Child [parent] = j ;
		}   
	    }
	}
    }

#ifndef NDEBUG
    {
	Int nels, ff, nchild ;
	DEBUG1 (("\n\n================================ ccolamd_postorder:\n"));
	nels = 0 ;
	for (j = 0 ; j < nn ; j++)
	{
	    if (Nv [j] > 0)
	    {
		DEBUG1 ((""ID" :  nels "ID" npiv "ID" size "ID
		    " parent "ID" maxfr "ID"\n", j, nels,
		    Nv [j], Fsize [j], Parent [j], Fsize [j])) ;
		/* this is an element */
		/* dump the link list of children */
		nchild = 0 ;
		DEBUG1 (("    Children: ")) ;
		for (ff = Child [j] ; ff != EMPTY ; ff = Sibling [ff])
		{
		    DEBUG1 ((ID" ", ff)) ;
		    nchild++ ;
		    ASSERT (nchild < nn) ;
		}
		DEBUG1 (("\n")) ;
		parent = Parent [j] ;
		nels++ ;
	    }
	}
    }
#endif

    /* --------------------------------------------------------------------- */
    /* place the largest child last in the list of children for each node */
    /* --------------------------------------------------------------------- */

    for (i = 0 ; i < nn ; i++)
    {
	if (Nv [i] > 0 && Child [i] != EMPTY)
	{

#ifndef NDEBUG
	    Int nchild ;
	    DEBUG1 (("Before partial sort, element "ID"\n", i)) ;
	    nchild = 0 ;
	    for (f = Child [i] ; f != EMPTY ; f = Sibling [f])
	    {
		DEBUG1 (("      f: "ID"  size: "ID"\n", f, Fsize [f])) ;
		nchild++ ;
	    }
#endif

	    /* find the biggest element in the child list */
	    fprev = EMPTY ;
	    maxfrsize = EMPTY ;
	    bigfprev = EMPTY ;
	    bigf = EMPTY ;
	    for (f = Child [i] ; f != EMPTY ; f = Sibling [f])
	    {
		frsize = Fsize [f] ;
		if (frsize >= maxfrsize)
		{
		    /* this is the biggest seen so far */
		    maxfrsize = frsize ;
		    bigfprev = fprev ;
		    bigf = f ;
		}
		fprev = f ;
	    }

	    fnext = Sibling [bigf] ;

	    DEBUG1 (("bigf "ID" maxfrsize "ID" bigfprev "ID" fnext "ID
		" fprev " ID"\n", bigf, maxfrsize, bigfprev, fnext, fprev)) ;

	    if (fnext != EMPTY)
	    {
		/* if fnext is EMPTY then bigf is already at the end of list */

		if (bigfprev == EMPTY)
		{
		    /* delete bigf from the element of the list */
		    Child [i] = fnext ;
		}
		else
		{
		    /* delete bigf from the middle of the list */
		    Sibling [bigfprev] = fnext ;
		}

		/* put bigf at the end of the list */
		Sibling [bigf] = EMPTY ;
		Sibling [fprev] = bigf ;
	    }

#ifndef NDEBUG
	    DEBUG1 (("After partial sort, element "ID"\n", i)) ;
	    for (f = Child [i] ; f != EMPTY ; f = Sibling [f])
	    {
		DEBUG1 (("        "ID"  "ID"\n", f, Fsize [f])) ;
		nchild-- ;
	    }
#endif
	}
    }

    /* --------------------------------------------------------------------- */
    /* postorder the assembly tree */
    /* --------------------------------------------------------------------- */

    for (i = 0 ; i < nn ; i++)
    {
	Order [i] = EMPTY ;
    }

    k = 0 ;

    for (i = 0 ; i < nn ; i++)
    {
	if ((Parent [i] == EMPTY
	    || (CMEMBER (Front_cols [Parent [i]]) != CMEMBER (Front_cols [i])))
	    && Nv [i] > 0)
	{
	    DEBUG1 (("Root of assembly tree "ID"\n", i)) ;
	    k = CCOLAMD_post_tree (i, k, Child, Sibling, Order, Stack) ;
	}
    }
}


/* ========================================================================= */
/* === CCOLAMD_post_tree =================================================== */
/* ========================================================================= */

/* Post-ordering of a supernodal column elimination tree.  */

GLOBAL Int CCOLAMD_post_tree
(
    Int root,			/* root of the tree */
    Int k,			/* start numbering at k */
    Int Child [ ],		/* input argument of size nn, undefined on
				 * output.  Child [i] is the head of a link
				 * list of all nodes that are children of node
				 * i in the tree. */
    const Int Sibling [ ],	/* input argument of size nn, not modified.
				 * If f is a node in the link list of the
				 * children of node i, then Sibling [f] is the
				 * next child of node i.
				 */
    Int Order [ ],		/* output order, of size nn.  Order [i] = k
				 * if node i is the kth node of the reordered
				 * tree. */
    Int Stack [ ]		/* workspace of size nn */
)
{
    Int f, head, h, i ;

#if 0
    /* --------------------------------------------------------------------- */
    /* recursive version (Stack [ ] is not used): */
    /* --------------------------------------------------------------------- */

    /* this is simple, but can cause stack overflow if nn is large */
    i = root ;
    for (f = Child [i] ; f != EMPTY ; f = Sibling [f])
    {
	k = CCOLAMD_post_tree (f, k, Child, Sibling, Order, Stack, nn) ;
    }
    Order [i] = k++ ;
    return (k) ;
#endif

    /* --------------------------------------------------------------------- */
    /* non-recursive version, using an explicit stack */
    /* --------------------------------------------------------------------- */

    /* push root on the stack */
    head = 0 ;
    Stack [0] = root ;

    while (head >= 0)
    {
	/* get head of stack */
	i = Stack [head] ;
	DEBUG1 (("head of stack "ID" \n", i)) ;

	if (Child [i] != EMPTY)
	{
	    /* the children of i are not yet ordered */
	    /* push each child onto the stack in reverse order */
	    /* so that small ones at the head of the list get popped first */
	    /* and the biggest one at the end of the list gets popped last */
	    for (f = Child [i] ; f != EMPTY ; f = Sibling [f])
	    {
		head++ ;
	    }
	    h = head ;
	    for (f = Child [i] ; f != EMPTY ; f = Sibling [f])
	    {
		ASSERT (h > 0) ;
		Stack [h--] = f ;
		DEBUG1 (("push "ID" on stack\n", f)) ;
	    }
	    ASSERT (Stack [h] == i) ;

	    /* delete child list so that i gets ordered next time we see it */
	    Child [i] = EMPTY ;
	}
	else
	{
	    /* the children of i (if there were any) are already ordered */
	    /* remove i from the stack and order it.  Front i is kth front */
	    head-- ;
	    DEBUG1 (("pop "ID" order "ID"\n", i, k)) ;
	    Order [i] = k++ ;
	}

#ifndef NDEBUG
	DEBUG1 (("\nStack:")) ;
	for (h = head ; h >= 0 ; h--)
	{
	    Int j = Stack [h] ;
	    DEBUG1 ((" "ID, j)) ;
	}
	DEBUG1 (("\n\n")) ;
#endif

    }
    return (k) ;
}



/* ========================================================================== */
/* === CCOLAMD debugging routines =========================================== */
/* ========================================================================== */

/* When debugging is disabled, the remainder of this file is ignored. */

#ifndef NDEBUG


/* ========================================================================== */
/* === debug_structures ===================================================== */
/* ========================================================================== */

/*
 *  At this point, all empty rows and columns are dead.  All live columns
 *  are "clean" (containing no dead rows) and simplicial (no supercolumns
 *  yet).  Rows may contain dead columns, but all live rows contain at
 *  least one live column.
 */

PRIVATE void debug_structures
(
    /* === Parameters ======================================================= */

    Int n_row,
    Int n_col,
    CColamd_Row Row [ ],
    CColamd_Col Col [ ],
    Int A [ ],
    Int cmember [ ],
    Int cset_start [ ]
)
{
    /* === Local variables ================================================== */

    Int i ;
    Int c ;
    Int *cp ;
    Int *cp_end ;
    Int len ;
    Int score ;
    Int r ;
    Int *rp ;
    Int *rp_end ;
    Int deg ;
    Int cs ;

    /* === Check A, Row, and Col ============================================ */

    for (c = 0 ; c < n_col ; c++)
    {
	if (COL_IS_ALIVE (c))
	{
	    len = Col [c].length ;
	    score = Col [c].shared2.score ;
	    DEBUG4 (("initial live col %5d %5d %5d\n", c, len, score)) ;
	    ASSERT (len > 0) ;
	    ASSERT (score >= 0) ;
	    ASSERT (Col [c].shared1.thickness == 1) ;
	    cp = &A [Col [c].start] ;
	    cp_end = cp + len ;
	    while (cp < cp_end)
	    {
		r = *cp++ ;
		ASSERT (ROW_IS_ALIVE (r)) ;
	    }
	}
	else
	{
	    i = Col [c].shared2.order ;
	    cs = CMEMBER (c) ;
	    ASSERT (i >= cset_start [cs] && i < cset_start [cs+1]) ;
	}
    }

    for (r = 0 ; r < n_row ; r++)
    {
	if (ROW_IS_ALIVE (r))
	{
	    i = 0 ;
	    len = Row [r].length ;
	    deg = Row [r].shared1.degree ;
	    ASSERT (len > 0) ;
	    ASSERT (deg > 0) ;
	    rp = &A [Row [r].start] ;
	    rp_end = rp + len ;
	    while (rp < rp_end)
	    {
		c = *rp++ ;
		if (COL_IS_ALIVE (c))
		{
		    i++ ;
		}
	    }
	    ASSERT (i > 0) ;
	}
    }
}


/* ========================================================================== */
/* === debug_deg_lists ====================================================== */
/* ========================================================================== */

/*
 *  Prints the contents of the degree lists.  Counts the number of columns
 *  in the degree list and compares it to the total it should have.  Also
 *  checks the row degrees.
 */

PRIVATE void debug_deg_lists
(
    /* === Parameters ======================================================= */

    Int n_row,
    Int n_col,
    CColamd_Row Row [ ],
    CColamd_Col Col [ ],
    Int head [ ],
    Int min_score,
    Int should,
    Int max_deg
)

{
    /* === Local variables ================================================== */

    Int deg ;
    Int col ;
    Int have ;
    Int row ;

    /* === Check the degree lists =========================================== */

    if (n_col > 10000 && ccolamd_debug <= 0)
    {
	return ;
    }
    have = 0 ;
    DEBUG4 (("Degree lists: "ID"\n", min_score)) ;
    for (deg = 0 ; deg <= n_col ; deg++)
    {
	col = head [deg] ;
	if (col == EMPTY)
	{
	    continue ;
	}
	DEBUG4 (("%d:", deg)) ;
	ASSERT (Col [col].shared3.prev == EMPTY) ;
	while (col != EMPTY)
	{
	    DEBUG4 ((" "ID"", col)) ;
	    have += Col [col].shared1.thickness ;
	    ASSERT (COL_IS_ALIVE (col)) ;
	    col = Col [col].shared4.degree_next ;
	}
	DEBUG4 (("\n")) ;
    }
    DEBUG4 (("should "ID" have "ID"\n", should, have)) ;
    ASSERT (should == have) ;

    /* === Check the row degrees ============================================ */

    if (n_row > 10000 && ccolamd_debug <= 0)
    {
	return ;
    }
    for (row = 0 ; row < n_row ; row++)
    {
	if (ROW_IS_ALIVE (row))
	{
	    ASSERT (Row [row].shared1.degree <= max_deg) ;
	}
    }
}


/* ========================================================================== */
/* === debug_mark =========================================================== */
/* ========================================================================== */

/*
 *  Ensures that the tag_mark is less that the maximum and also ensures that
 *  each entry in the mark array is less than the tag mark.
 */

PRIVATE void debug_mark
(
    /* === Parameters ======================================================= */

    Int n_row,
    CColamd_Row Row [ ],
    Int tag_mark,
    Int max_mark
)
{
    /* === Local variables ================================================== */

    Int r ;

    /* === Check the Row marks ============================================== */

    ASSERT (tag_mark > 0 && tag_mark <= max_mark) ;
    if (n_row > 10000 && ccolamd_debug <= 0)
    {
	return ;
    }
    for (r = 0 ; r < n_row ; r++)
    {
	ASSERT (Row [r].shared2.mark < tag_mark) ;
    }
}


/* ========================================================================== */
/* === debug_matrix ========================================================= */
/* ========================================================================== */

/* Prints out the contents of the columns and the rows.  */

PRIVATE void debug_matrix
(
    /* === Parameters ======================================================= */

    Int n_row,
    Int n_col,
    CColamd_Row Row [ ],
    CColamd_Col Col [ ],
    Int A [ ]
)
{
    /* === Local variables ================================================== */

    Int r ;
    Int c ;
    Int *rp ;
    Int *rp_end ;
    Int *cp ;
    Int *cp_end ;

    /* === Dump the rows and columns of the matrix ========================== */

    if (ccolamd_debug < 3)
    {
	return ;
    }
    DEBUG3 (("DUMP MATRIX:\n")) ;
    for (r = 0 ; r < n_row ; r++)
    {
	DEBUG3 (("Row "ID" alive? "ID"\n", r, ROW_IS_ALIVE (r))) ;
	if (ROW_IS_DEAD (r))
	{
	    continue ;
	}

	DEBUG3 (("start "ID" length "ID" degree "ID"\nthickness "ID"\n",
		Row [r].start, Row [r].length, Row [r].shared1.degree,
		Row [r].thickness)) ;

	rp = &A [Row [r].start] ;
	rp_end = rp + Row [r].length ;
	while (rp < rp_end)
	{
	    c = *rp++ ;
	    DEBUG4 (("	"ID" col "ID"\n", COL_IS_ALIVE (c), c)) ;
	}
    }

    for (c = 0 ; c < n_col ; c++)
    {
	DEBUG3 (("Col "ID" alive? "ID"\n", c, COL_IS_ALIVE (c))) ;
	if (COL_IS_DEAD (c))
	{
	    continue ;
	}
	DEBUG3 (("start "ID" length "ID" shared1 "ID" shared2 "ID"\n",
		Col [c].start, Col [c].length,
		Col [c].shared1.thickness, Col [c].shared2.score)) ;
	cp = &A [Col [c].start] ;
	cp_end = cp + Col [c].length ;
	while (cp < cp_end)
	{
	    r = *cp++ ;
	    DEBUG4 (("	"ID" row "ID"\n", ROW_IS_ALIVE (r), r)) ;
	}
    }
}


/* ========================================================================== */
/* === dump_super =========================================================== */
/* ========================================================================== */

PRIVATE void dump_super
(
    Int super_c,
    CColamd_Col Col [ ],
    Int n_col
)
{
    Int col, ncols ;

    DEBUG1 ((" =[ ")) ;
    ncols = 0 ;
    for (col = super_c ; col != EMPTY ; col = Col [col].nextcol)
    {
        DEBUG1 ((" "ID, col)) ;
        ASSERT (col >= 0 && col < n_col) ;
        if (col != super_c)
        {
            ASSERT (COL_IS_DEAD (col)) ;
        }
        if (Col [col].nextcol == EMPTY)
        {
            ASSERT (col == Col [super_c].lastcol) ;
        }
        ncols++ ;
        ASSERT (ncols <= Col [super_c].shared1.thickness) ;
    }
    ASSERT (ncols == Col [super_c].shared1.thickness) ;
    DEBUG1 (("]\n")) ;
}


/* ========================================================================== */
/* === ccolamd_get_debug ==================================================== */
/* ========================================================================== */

PRIVATE void ccolamd_get_debug
(
    char *method
)
{
    FILE *debug_file ;
    ccolamd_debug = 0 ;		/* no debug printing */

    /* Read debug info from the debug file. */
    debug_file = fopen ("debug", "r") ;
    if (debug_file)
    {
	(void) fscanf (debug_file, ""ID"", &ccolamd_debug) ;
	(void) fclose (debug_file) ;
    }

    DEBUG0 ((":")) ;
    DEBUG1 (("%s: debug version, D = "ID" (THIS WILL BE SLOW!)\n",
    	method, ccolamd_debug)) ;
    DEBUG1 ((" Debug printing level: "ID"\n", ccolamd_debug)) ;
}

#endif
