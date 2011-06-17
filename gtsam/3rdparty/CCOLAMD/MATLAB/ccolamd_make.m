function ccolamd_make
%CCOLAMD_MAKE compiles CCOLAMD and CSYMAMD for MATLAB
%
% Example:
%   ccolamd_make
%
% See also ccolamd, csymamd

% Copyright 1998-2007, Timothy A. Davis, Stefan Larimore, and Siva Rajamanickam
% Developed in collaboration with J. Gilbert and E. Ng.

details = 0 ;	    % 1 if details of each command are to be printed
d = '' ;
if (~isempty (strfind (computer, '64')))
    d = '-largeArrayDims' ;
end
src = '../Source/ccolamd.c ../Source/ccolamd_global.c' ;
cmd = sprintf ('mex -DDLONG -O %s -I../../UFconfig -I../Include -output ', d) ;
s = [cmd 'ccolamd ccolamdmex.c ' src] ;
if (details)
    fprintf ('%s\n', s) ;
end
eval (s) ;
s = [cmd 'csymamd csymamdmex.c ' src] ;
if (details)
    fprintf ('%s\n', s) ;
end
eval (s) ;
fprintf ('CCOLAMD and CSYMAMD successfully compiled.\n') ;
