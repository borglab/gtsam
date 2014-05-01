@echo off

REM This batch file compiles the lpsolve libraries with the Microsoft Visual C/C++ compiler under Windows

set src=../lp_MDO.c ../shared/commonlib.c ../shared/mmio.c ../shared/myblas.c ../ini.c ../colamd/colamd.c ../lp_rlp.c ../lp_crash.c ../bfp/bfp_LUSOL/lp_LUSOL.c ../bfp/bfp_LUSOL/LUSOL/lusol.c ../lp_Hash.c ../lp_lib.c ../lp_wlp.c ../lp_matrix.c ../lp_mipbb.c ../lp_MPS.c ../lp_params.c ../lp_presolve.c ../lp_price.c ../lp_pricePSE.c ../lp_report.c ../lp_scale.c ../lp_simplex.c ../lp_SOS.c ../lp_utils.c ../yacc_read.c

set c=cl

rc lpsolve.rc
rem use /MT to remove dependence on msvcrt80.dll calls kernel libs directly about 200K larger dll.
rem %c% -W1 -I.. -I../shared -I../bfp -I../bfp/bfp_LUSOL -I../bfp/bfp_LUSOL/LUSOL -I../colamd /LD /MD /O2 /Zp8 /Gz -D_WINDLL -D_USRDLL -DWIN32 -D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE -DYY_NEVER_INTERACTIVE -DPARSER_LP -DINVERSE_ACTIVE=INVERSE_LUSOL -DRoleIsExternalInvEngine %src% lpsolve.res ..\lp_solve.def  /Felpsolve55.dll
    %c% -W1 -I.. -I../shared -I../bfp -I../bfp/bfp_LUSOL -I../bfp/bfp_LUSOL/LUSOL -I../colamd /LD /MT /O2 /Zp8 /Gz -D_WINDLL -D_USRDLL -DWIN32 -D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE -DYY_NEVER_INTERACTIVE -DPARSER_LP -DINVERSE_ACTIVE=INVERSE_LUSOL -DRoleIsExternalInvEngine %src% lpsolve.res ..\lp_solve.def  /Felpsolve55.dll

rem http://msdn2.microsoft.com/en-us/library/ms235229.aspx
rem for vs2005 need to embed manifest in dll with manifest tool #2 on the next line does this.
rem mt /outputresource:".\lpsolve55.dll;#2" /manifest ".\lpsolve55.dll.manifest"
rem pause

if exist a.obj del a.obj
%c% -I.. -I../shared -I../bfp -I../bfp/bfp_LUSOL -I../bfp/bfp_LUSOL/LUSOL -I../colamd /MT /O2 /Zp8 /Gd /c -DWIN32 -D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE -DYY_NEVER_INTERACTIVE -DPARSER_LP -DINVERSE_ACTIVE=INVERSE_LUSOL -DRoleIsExternalInvEngine %src%
lib *.obj /OUT:liblpsolve55.lib

if exist a.obj del a.obj
%c% -I.. -I../shared -I../bfp -I../bfp/bfp_LUSOL -I../bfp/bfp_LUSOL/LUSOL -I../colamd /MTd /Od /Zp8 /Gd /RTC1 /c -DWIN32 -D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE -DYY_NEVER_INTERACTIVE -DPARSER_LP -DINVERSE_ACTIVE=INVERSE_LUSOL -DRoleIsExternalInvEngine %src%
lib *.obj /OUT:liblpsolve55d.lib

if exist *.obj del *.obj
