README.TXT for LUSOL - Advanced LU solver with enhanced numerical stability options
-----------------------------------------------------------------------------------

LUSOL - pronounced "L-U-SOL" - was developed by Prof. Michael Saunders at the 
Stanford (University) Optimization Laboratory over a period of 2 decades of 
progressive improvements.  It is a particularly capable matrix factorization system
and includes sparsity-preserving column updates and equation solving.  It is
therefore particularly well suited to be part of a system to solve tough mathematical 
programming problems.  Further details can be found in the file "LUSOL-overview.txt."

A big step has been made in converting the original Fortran code into a much more
easily accessible and modularized system based on ANSI C as part of the release of
lp_solve v5.  LUSOL is fully implemented as a "Basis Factorization Package", BFP in 
lp_solve and is the BFP of choice for large and complex models, if not all.  As part
of the conversion to C, processor optimized BLAS functionality has been enabled, and
future enhancements to LUSOL may make increasing use of this, ensuring top performance. 

For the lp_solve release of LUSOL, a stand-alone equation solving system has also been
developed.  A pre-compiled Windows command-line executable version is included in the
standard distribution of LUSOL. In addition, the program options illustrate several 
advanced uses of LUSOL.  The equation solving utility features reading of standard 
matrix files in the Harwell-Boeing, MatrixMarket and text formats.  Sample matrix 
models are provided for Harwell-Boeing (.RUA) and MatrixMarket (.mtx).  

The LUSOL code is released under the GNU Lesser General Public Licence.  Confer the 
file "Licence_LGPL.txt" for the full terms of this licence.  These terms make lp_solve
and LUSOL available and distributable on equal licencing terms.  It is expected that
LUSOL will have an official repository in the near future, but the LUSOL archive at
the Yahoo lp_solve group will be an official copy and the formal repository until
further notice.


Kjell Eikland
14 July 2004
Oslo, Norway
