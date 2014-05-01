Introduction
------------
What is lp_solve and what is it not?
The simple answer is, lp_solve is a Mixed Integer Linear Programming (MILP) solver.

It is a free (see LGPL for the GNU lesser general public license) linear (integer) programming solver
based on the revised simplex method and the Branch-and-bound method for the integers.
It contains full source, examples and manuals.
lp_solve solves pure linear, (mixed) integer/binary, semi-continuous and
special ordered sets (SOS) models.

See the reference guide for more information.


lp_solve 5.5
------------

Why a jump from version numbers 5.1 to 5.5 ?
This is done to indicate that this is more than just another update.
The solver engine was revised and optimised in such a way that performance has improved considerably.
Numerical stability is also better resulting in more models that can be solved.
The LUSOL bfp is now also the default. In the past, the etaPFI bfp package was the default,
but for larger models this leads faster to numerical instabilities and performance problems.

Overall, the v5.5 code is faster and more robust than v5.1.
This robustness is for example proven by the fact that many more models can now be solved even without scaling.

The API hasn't changed very much.
There are a couple of new routines and one routine has an extra argument.
Some constants got new values.

    * Fundamental internal change to the solver engine resulting in better performance and numerical stability.
      Both the LP solver and the B&B solvers are enhanced.
    * Optimised MILP branch truncation, with reduced cost fixing.
    * LUSOL bfp is now the default.
    * Presolve is improved in functionality and performance.
    * Better handling of degeneracy, with more options.
    * Store and read options from a file make it easier to set options.
    * Partial pricing for the primal simplex now works.
    * Full support for xli_ZIMPL v2.0.3.
    * The objective function is no longer stored as part of the constraint matrix.
    * Dual-long step code is in place, but not fully activated yet.
    * General code cleanup.
    * Added OBJSENSE and OBJNAME headers in the free MPS format (See MPS file format).
    * The MathProg xli driver has now the ability to generate a model.
    * New API routines

Start by taking a look at 'Changes compared to version 4', 'Changes from version 5.1 to version 5.5'
and 'lp_solve usage'
This gives a good starting point.


BFP's
-----

BFP stands for Basis Factorization Package, which is a unique lp_solve feature.  Considerable
effort has been put in this new feature and we have big expectations for this. BFP is a generic
interface model and users can develop their own implementations based on the provided templates.
We are very interested in providing as many different BFPs as possible to the community.

lp_solve 5.5 has the LUSOL BFP built in as default engine.  In addition two other
BFPs are included for both Windows and Linux: bfp_etaPFI.dll, bfp_GLPK.dll for Windows and
libbfp_etaPFI.so, libbfp_GLPK.so for Linux.  The bfp_etaPFI includes
advanced column ordering using the COLAMD library, as well as better pivot management for
stability.  For complex models, however, the LU factorization approach is much better, and
lp_solve now includes LUSOL as one of the most stable engines available anywhere.  LUSOL was
originally developed by Prof. Saunders at Stanford, and it has now been ported to C
and enhanced by Kjell.

If you compile BFPs yourself, make sure that under Windows, you use __stdcall convention and
use 8 byte alignments.  This is needed for the BFPs to work correctly with the general
distribution of lp_solve and also to make sharing BFPs as uncomplicated as possible.

See the reference guide for more information.


XLI's
-----

XLI stands for eXternal Language Interface, also a unique lp_solve feature. XLI's are stand-alone
libraries used as add-on to lp_solve to make it possible to read and write lp models in a format
not natively supported by lp_solve. Examples are CPLEX lp format, LINDO lp format, MathProg format,
XML format...

See the reference guide for more information.


lpsolve API
-----------

Don't forget that the API has changed compared to previous versions of lpsolve and that you just
can't use the version 5 lpsolve library with your version 4 or older code.  That is also the
reason why the library is now called lpsolve55.dll/lpsolve55.a.  lpsolve55.dll or lpsolve55.a are
only needed when you call the lpsolve library via the API interface from your program.
The lp_solve program is still a stand-alone executable program.

There are examples interfaces for different language like C, VB, C#, VB.NET, Java,
Delphi, and there is now also even a COM object to access the lpsolve library.  This means that
the door is wide-open for using lp_solve in many different situations.  Thus everything that is
available in version 4 is now also available in version 5 and already much more!

See the reference guide for more information.


Conversion between lp modeling formats
--------------------------------------

Note that lp2mps and mps2lp don't exist anymore. However this functionality is now implemented
in lp_solve:

lp2mps can be simulated as following:
lp_solve -parse_only -lp infile -wmps outfile

mps2lp can be simulated as following:
lp_solve -parse_only -mps infile -wlp outfile


via the -rxli option, a model can be read via an XLI library and via the -wxli option, a model
can be written via an XLI library.


How to build the executables yourself.
---------------------------------------

At this time, there are no Makefiles yet. However for the time being, there are batch files/scripts
to build. For the Microsoft compiler under Windows, use cvc6.bat, for the gnu compiler under Windows,
use cgcc.bat and for Unix/Linux, use the ccc shell script (sh ccc).

See the reference guide for more information.


IDE
---

Under Windows, there is now also a very user friendly lpsolve IDE. Check out LPSolveIDE

See the reference guide for more information.


Documentation (reference guide)
-------------------------------

See lp_solve55.chm for a Windows HTML help documentation file.
The html files are also in lp_solve_5.5_doc.tar.gz. Start with index.htm
Also see http://lpsolve.sourceforge.net/ for a on-line documentation


Change history:
---------------

17/05/05 version 5.5.0.0
- Beta release of version 5.5

??/??/05 version 5.5.0.1
- ?

26/06/05 version 5.5.0.2
- ?

29/06/05 version 5.5.0.3
- ?

16/08/05 version 5.5.0.4
- There are no API changes
- The LUSOL message routine could generate a crash under some cicumstances. Fixed
- A crash could occur when building the model in add_row_mode. Fixed.
- write_params didn't write the PRESOLVE and PRESOLVELOOPS correctly. Fixed.
- write_params didn't write constants with value 0. Fixed.
- The library did not compile under msdev 2002 (VC 7.0 _MSC_VER 1300). Fixed.
- There were some problems with printing long long variables which could generate a crash. Fixed.
- An overflow error could occur because memory was sometimes overwritten. Fixed.
- Presolve routines are revised. They are again improved and made faster.
  Also some problems with it are fixed (possible crashes).
- Solver revised. Again made faster and more stable.
- get_row/get_column returned FALSE if the row/column is empty. Fixed.
- get_rowex/get_columnex now returns -1 if and error is detected. This instead of 0.
  This to know the distinction between an empty row/column and an error.
- set_bounds had a possible problem when min and max are equal. Fixed.
- A crash/damage error could occur when rows/columns are added after a solve. Fixed.
- The my_chsign macro in lp_types.h gave warnings with some compilers. Fixed.
- The lp_solve program now returns 255 if an unexpected error occurs. Before this was 1
  But this interferes with the lpsolve library return codes.
- With the lp_solve program, debug and print modes were not written correctly in a
  specified parameter file. Fixed.
- With the lp_solve program, presolveloops was not set correctly. Fixed.

17/09/05 version 5.5.0.5
- In some cases, SOS restrictions were not optimized till optimality. Fixed.
- Presolve sometimes generated 'Column -xxx out of range during presolve' with a possible crash.
- Presolve sometimes removed integer and/or semi-cont variables that should not be deleted. Fixed.
- B&B sometimes didn't find the most optimal solution. Fixed.
- Internal constant COMP_EQUAL renamed to COMP_PREFERNONE because this could interfere with a define
  in the standard header files.
- The lp parser had problems with variables starting with INF and there is a + or - sign just before it.
  Fixed.
- Added options -presolvem, -presolvefd, -presolvebnd, -presolved, -presolveslk
- Updated documentation. put_bb_branchfunc, put_bb_nodefunc, set_epslevel, dualize_lp, set_basisvar

16/11/05 version 5.5.0.6
- set_add_rowmode should not be called after a solve. There is now a test in this routine when this is
  done and it then returns FALSE.
- When an empty string ("") as filename is provided to set_outputfile, then output is completely
  ignored.
- In some cases, SOS models did not solve to their most optimal solution.
- There was as problem with get_sensitivity_objex. Calling it (sometimes only after multiple times)
  resulted in protection errors/core dumps.
- When a model has no constraints, it did not solve most of the times.
- column_in_lp didn't work anymore.
- Large upper bounds could make the model unstable. A change was made to fix this.
- set_improve could let crash the model.
- lp_params.c used the non-ANSI function unlink(). Changed to ANSI function remove().
- Presolve is again revised considerably.
- SOS handling is improved when there are a lot of SOS constraints.
- Limited constraint-to-SOS1 presolve to constraints with at least 4 variables.
- Limited bound tightening presolve loops.

12/02/06 version 5.5.0.7
- When SOS restrictions are added after a previous solve, a crash could occur.
- Optimized renaming a variable when the new name is equal to the old name.
- A possible crash could occur when presolve was enabled
- The constant ANTIDEGEN_DEFAULT is changed. ANTIDEGEN_INFEASIBLE is removed from it.
  This constant should not be used unless you have some very tight and hard to solve
  models where the optimal solution numerically straddles infeasibility.
- There was a possible problem with set_row(ex). It sometimes wrongfully changed the row.
- When integer variables were scaled, it could happen that because of rounding errors,
  a loop was created.
- Sometimes integer models kept on looping in the B&B algorithm.
- A memory overrun could occur when an initial basis is set. This when variable names
  are in Rnnn format and constraint names in Cnnn format.
- Some fixes are made in presolve.
- On 64-bit systems, compiler warnings were given and some code worked wrong resulting in
  wrong results.
- lp_solve.c didn't compile with some compilers because if a very deep nested if statement.
- The distributed files now have the version number include in the filename.
  For example lp_solve_5.5.0.7_exe.zip
  This for a possible move to SourceForge in the (near?) future.
- When illegal bounds are specified in the MPS format (lower bound larger than upper bound)
  then a warning was given but the illegal bound was just ignored and the model was solved.
  This resulted in a solution that did not comply to the original model. Now the message is
  seen as an error and solving is aborted.


06/09/06 version 5.5.0.8
- When presolve is active and columns are removed and there are SOS constraints, then presolve
  had an error which could result in hanging while solve or maybe wrong solutions.
- set_row(ex) set wrong values when used after a previous solve and scaling is active.
- disabled PRESOLVE_REDUCEMIP since it is very rare that this is effective, and also that it adds
  code complications and delayed presolve effects that are not captured properly.
- made routine guess_basis available for all languages (now exported by the dll).
  The routine is now also documented.
- some bug corrections in guess_basis.
- Corrected a problem with add_column(ex) when add_rowmode is enabled.
- write_lp now wraps long lines over multiple lines to make it more readable.
- A compilation warning/error sometimes occured on is_fixedvar in lp_lusol.c with some compilers.
- Added options -wxlisol and -wxlisolopt to lp_solve program to write a solution file for those
  XLIs that support it.
- Updated CPLEX XLI to support constants in objective.
- Added documentation on infeasible models, guess_basis, DIMACS models, CPLEX format, Zimpl, GNU Mathprog.
  Corrected/updated documentation on get_col_name, get_row_name, get_nameindex, write_xli,
  External Language Interfaces.
- The mps reader was improved for the rarely cases where the same elements are provided multiple
  times. These are now added.
- Revised the java unittest example because it gave some errors that it shouldn't give.

07/10/06 version 5.5.0.9
- set_row(ex) could sometimes set wrong values in the model.
- Sometimes models with semi-cont variables which are also integer and scaling is active, a solution
  was returned that is not most optimal or it returns infeasible.
- write_mps didn't write semi-cont variables with an infinite upper bound.
- When presolve can solve the model on its own and objective direction is maximize then a wrong sign
  was shown in the reported price on screen.
- write_lp writes constraint and bounds in the same way if a constraint is not named. If a constraint
  only has one variable then it looks like a bound. This can give problems because when a constraint
  is interpreted as bound and it is negative then the problem definition changes.
  Therefore a constraint which is not named and having only one variable in it is getting a name to
  make sure it is interpreted as a constraint.
- The lp_solve program didn't interprete the PRESOLVED solve return code very well. Fixed.
- bfp_GLPK and xli_MathProg are now compiled against GLPK 4.11
- When an integer model is aborted before the most optimal solution is found (timeout or
  break at first, ...) solve returned OPTIMAL (0) instead of SUBOPTIMAL (1). This is now corrected.

14/01/07 version 5.5.0.10
- If a model has integer variables, but the model is already integer optimal in the simplex fase,
  then it was reported as suboptimal feasible.
- get_objective, get_variables, get_ptr_variables, get_constraints, get_ptr_constraints, get_primal_solution
  reported 'not a valid basis' when presolve is active and the model is entirely solved by presolve.
- presolve on a model with SOS variables sometimes went wrong.
- presolve on a model with SOS variables where infeasibility is detected crashed.
- read_bas could fail when not all constraints had names or had names like default variable names.
- A crash could occur with set_row(ex) in rowmode.
- The lp format has been extended with a free section to define free variables.
- bfp_GLPK and xli_MathProg are now compiled against GLPK 4.13
- fixed bug in the pseudocost logic that can blow up subsequent pseudocost values in that
  branch and make them almost random.
- In some rare cases a memory overrun could occur when constraints are added after a previous solve.
- Made the copy_lp routine work. Note that information about the optimisation of the original model
  is not copied (at this time). Only the model is.
- Fixed a bug in the hashing routines that had only influence in some rare cases in the
  MATLAB, O-Matrix, Scilab, Octave, Python drivers.
- coldual sometimes worked on a uninitialised variable value with unpredictable results.

27/12/07 version 5.5.0.11
- Fixed a problem in presolve. Sometimes an array-index-out-of-bounds error occured.
- Added a makefile for Linux.
- When adding constraints, in some rare cases a memory overrun could occur resulting in a crash.
- add_constraintex with count=0 and row=colno=NULL gave a protection error.
  several XLIs didn't work anymore because of this.
- set_constr_type sometimes set wrong signs for the coefficient matrix when add_rowmode is on.
- presolve did an unnecessary extra loop. Also a test is added to stop presolve when very few
  changes are done.
- for very large models, a request of much more memory than is reasonable could occur. Fixed.
- Modified LINDO XLI to read keywords also not at column 1 and to accept an empty objective function.
  Previously this wat not possible.
- In some rare cases, numbers very close to their integer values (for example 11276.999999999998)
  were truncated to their ceiling value (for example 11276) instead of rounded
  (for example 11277).
- Solved a problem with presolve with an all-int constraint.
- Solved a problem with presolve coldominate
- Added stronger checking of the MPS format.
  Fields that must be blank are now also tested accordingly so that if data is there that it is
  not ignored as before.
- FREE MPS format was not read ok if row/column name were all numbers
  or a FR, MI, PL, BV bound was defined. Fixed.
- The lp-format now also supports a bin(ary) section to define binary variables.
- When an integer model is aborted before the most optimal solution is found
  via break at first or break at value, solve returned OPTIMAL (0) instead of SUBOPTIMAL (1).
  This is now corrected. Problem occured in version 5.5.0.10
- Fixed a problem with del_constraint. Constraints names were not shifted and reported variable result was incorrect.
- read_XLI failed with MathProg if datamodel was provided with "" when there is no datamodel.
  NULL was expected in the past. "" is now also accepted.
- Added an XLI to read Xpress lp files.
- Added routines MPS_writefileex, write_lpex.
- Added options -o0, -o1 to lp_solve command driven program to specify if objective is in basis or not.
- Added new information in the reference guide:
   - Linear programming basics
   - Presolve
   - Xpress lp files

We are thrilled to hear from you and your experiences with this new version. The good and the bad.
Also we would be pleased to hear about your experiences with the different BFPs on your models.

Please send reactions to:
Peter Notebaert: lpsolve@peno.be
Kjell Eikland: kjell.eikland@broadpark.no
