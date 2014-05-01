@echo off

REM This batch file compiles the demo program with the gnu gcc compiler under DOS/Windows

REM There are two ways to do that: use the lpsolve code directly or use that static library.

rem link lpsolve code with application
set src=../lp_MDO.c ../shared/commonlib.c ../colamd/colamd.c ../shared/mmio.c ../shared/myblas.c ../ini.c ../lp_rlp.c ../lp_crash.c ../bfp/bfp_LUSOL/lp_LUSOL.c ../bfp/bfp_LUSOL/LUSOL/lusol.c ../lp_Hash.c ../lp_lib.c ../lp_wlp.c ../lp_matrix.c ../lp_mipbb.c ../lp_MPS.c ../lp_params.c ../lp_presolve.c ../lp_price.c ../lp_pricePSE.c ../lp_report.c ../lp_scale.c ../lp_simplex.c ../lp_SOS.c ../lp_utils.c ../yacc_read.c

rem statically link lpsolve library
rem set src=../lpsolve55/liblpsolve55.a

set c=gcc

%c%  -DINLINE=static -I.. -I../bfp -I../bfp/bfp_LUSOL -I../bfp/bfp_LUSOL/LUSOL -I../colamd -I../shared -O3 -DBFP_CALLMODEL=__stdcall -DYY_NEVER_INTERACTIVE -DPARSER_LP -DINVERSE_ACTIVE=INVERSE_LUSOL -DRoleIsExternalInvEngine demo.c %src% -o demo.exe
