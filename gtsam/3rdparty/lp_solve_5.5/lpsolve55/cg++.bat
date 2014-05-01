@echo off

REM This batch file compiles the lpsolve libraries with the GNU gcc compiler under Windows


set src=../lp_MDO.c ../shared/commonlib.c ../shared/mmio.c ../shared/myblas.c ../ini.c ../colamd/colamd.c ../lp_rlp.c ../lp_crash.c ../bfp/bfp_LUSOL/lp_LUSOL.c ../bfp/bfp_LUSOL/LUSOL/lusol.c ../lp_Hash.c ../lp_lib.c ../lp_wlp.c ../lp_matrix.c ../lp_mipbb.c ../lp_MPS.c ../lp_params.c ../lp_presolve.c ../lp_price.c ../lp_pricePSE.c ../lp_report.c ../lp_scale.c ../lp_simplex.c ../lp_SOS.c ../lp_utils.c ../yacc_read.c

set c=g++

rem rc lpsolve.rc
%c% -DINLINE=static -I.. -I../shared -I../bfp -I../bfp/bfp_LUSOL -I../bfp/bfp_LUSOL/LUSOL -I../colamd -s -O3 -shared -mno-cygwin -enable-stdcall-fixup -D_USRDLL -DWIN32 -DYY_NEVER_INTERACTIVE -DPARSER_LP -DINVERSE_ACTIVE=INVERSE_LUSOL -DRoleIsExternalInvEngine %src% ..\lp_solve.def -o lpsolve55.dll

%c% -DINLINE=static -I.. -I../shared -I../bfp -I../bfp/bfp_LUSOL -I../bfp/bfp_LUSOL/LUSOL -I../colamd -s -O3 -shared -D_USRDLL -DWIN32 -DYY_NEVER_INTERACTIVE -DPARSER_LP -DINVERSE_ACTIVE=INVERSE_LUSOL -DRoleIsExternalInvEngine %src% -o liblpsolve55.so

if exist a.o del a.o
%c% -DINLINE=static -I.. -I../shared -I../bfp -I../bfp/bfp_LUSOL -I../bfp/bfp_LUSOL/LUSOL -I../colamd -s -O3 -c -DYY_NEVER_INTERACTIVE -DPARSER_LP -DINVERSE_ACTIVE=INVERSE_LUSOL -DRoleIsExternalInvEngine %src%
ar rv liblpsolve55.a *.o

if exist *.o del *.o
