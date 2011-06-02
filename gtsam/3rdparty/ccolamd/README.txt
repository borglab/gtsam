CCOLAMD: constrained column approximate minimum degree ordering
Copyright (C) 2005-2011, Univ. of Florida.  Authors: Timothy A. Davis,
Sivasankaran Rajamanickam, and Stefan Larimore.  Closely based on COLAMD by
Davis, Stefan Larimore, in collaboration with Esmond Ng, and John Gilbert.
http://www.cise.ufl.edu/research/sparse
-------------------------------------------------------------------------------

The CCOLAMD column approximate minimum degree ordering algorithm computes
a permutation vector P such that the LU factorization of A (:,P)
tends to be sparser than that of A.  The Cholesky factorization of
(A (:,P))'*(A (:,P)) will also tend to be sparser than that of A'*A.
CSYMAMD is a symmetric minimum degree ordering method based on CCOLAMD, also
available as a MATLAB-callable function.  It constructs a matrix M such
that M'*M has the same pattern as A, and then uses CCOLAMD to compute a column
ordering of M.

Requires UFconfig, in the ../UFconfig directory relative to this directory.

To compile and install the ccolamd m-files and mexFunctions, just cd to
CCOLAMD/MATLAB and type ccolamd_install in the MATLAB command window.
A short demo will run.  Optionally, type ccolamd_test to run an extensive tests.
Type "make" in Unix in the CCOLAMD directory to compile the C-callable
library and to run a short demo.

If you have MATLAB 7.2 or earlier, you must first edit UFconfig/UFconfig.h to
remove the "-largeArrayDims" option from the MEX command (or just use
ccolamd_install.m inside MATLAB).

Other "make" targets:

    make mex		compiles MATLAB mexFunctions only
    make libccolamd.a	compiles a C-callable library containing ccolamd
    make clean		removes all files not in the distribution, except for
			libccolamd.a
    make distclean	removes all files not in the distribution

To use ccolamd and csymamd within an application written in C, all you need are
ccolamd.c and ccolamd.h, which are the C-callable ccolamd/csymamd codes.
See ccolamd.c for more information on how to call ccolamd from a C program.
It contains a complete description of the C-interface to CCOLAMD and CSYMAMD.

	Copyright (c) 1998-2007 by the University of Florida.
	All Rights Reserved.

	Licensed under the GNU LESSER GENERAL PUBLIC LICENSE.

-------------------------------------------------------------------------------

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

-------------------------------------------------------------------------------


Related papers:

	T. A. Davis and W. W. Hager, Rajamanickam, Multiple-rank updates
	to a supernodal sparse Cholesky factorization, submitted.

	T. A. Davis, W. W. Hager, S. Rajamanickam, and Y. Chen, CHOLMOD: a
	sparse Cholesky update/downdate package, submitted.  CHOLMOD's
	nested dissection ordering relies on CCOLAMD and CSYMAMD to order
	the matrix after graph partitioning is used to find the ordering
	constraints.

	T. A. Davis, J. R. Gilbert, S. Larimore, E. Ng, An approximate column
	minimum degree ordering algorithm, ACM Transactions on Mathematical
	Software, vol. 30, no. 3., pp. 353-376, 2004.

	T. A. Davis, J. R. Gilbert, S. Larimore, E. Ng, Algorithm 836: COLAMD,
	an approximate column minimum degree ordering algorithm, ACM
	Transactions on Mathematical Software, vol. 30, no. 3., pp. 377-380,
	2004.

	"An approximate minimum degree column ordering algorithm",
	S. I. Larimore, MS Thesis, Dept. of Computer and Information
	Science and Engineering, University of Florida, Gainesville, FL,
	1998.  CISE Tech Report TR-98-016.  Available at 
	ftp://ftp.cise.ufl.edu/cis/tech-reports/tr98/tr98-016.ps
	via anonymous ftp.

	Approximate Deficiency for Ordering the Columns of a Matrix,
	J. L. Kern, Senior Thesis, Dept. of Computer and Information
	Science and Engineering, University of Florida, Gainesville, FL,
	1999.  Available at http://www.cise.ufl.edu/~davis/Kern/kern.ps 

Authors:  Timothy A. Davis, Sivasankaran Rajamanickam, and Stefan Larimore.
	Closely based on COLAMD by Stefan I. Larimore and Timothy A. Davis,
	University of Florida, in collaboration with John Gilbert, Xerox PARC
	(now at UC Santa Barbara), and Esmong Ng, Lawrence Berkeley National
	Laboratory (much of this work he did while at Oak Ridge National
	Laboratory). 

CCOLAMD files:

    Demo	    simple demo
    Doc		    additional documentation (see ccolamd.c for more)
    Include	    include file
    Lib		    compiled C-callable library
    Makefile	    primary Unix Makefile
    MATLAB	    MATLAB functions
    README.txt	    this file
    Source	    C source code

    ./Demo:
    ccolamd_example.c	    simple example
    ccolamd_example.out	    output of colamd_example.c
    ccolamd_l_example.c	    simple example, long integers
    ccolamd_l_example.out   output of colamd_l_example.c
    Makefile		    Makefile for C demos

    ./Doc:
    ChangeLog	    change log
    lesser.txt	    license

    ./Include:
    ccolamd.h	    include file

    ./Lib:
    Makefile	    Makefile for C-callable library

    ./MATLAB:
    ccolamd.m		MATLAB interface for ccolamd
    ccolamd_demo.m	simple demo
    ccolamd_install.m	compile and install ccolamd and csymamd
    ccolamd_make.m	compile colamd2 and symamd2
    ccolamdmex.c	MATLAB mexFunction for ccolamd
    ccolamd_test.m	extensive test
    ccolamdtestmex.c	test function for ccolamd
    Contents.m		contents of the MATLAB directory
    luflops.m		test code
    Makefile		Makefile for MATLAB functions
    csymamd.m		MATLAB interface for csymamd
    csymamdmex.c	MATLAB mexFunction for csymamd
    symamdtestmex.c	test function for csymamd

    ./Source:
    ccolamd.c		primary source code
    ccolamd_global.c	globally defined function pointers (malloc, free, ...)
