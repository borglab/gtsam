function ccolamd_install
%CCOLAMD_INSTALL compiles and installs ccolamd and csymamd for MATLAB
%   Your current directory must be CCOLAMD/MATLAB for this function to work.
%
% Example:
%   ccolamd_install
%
% See also ccolamd, csymamd.

% Copyright 1998-2007, Timothy A. Davis, Stefan Larimore, and Siva Rajamanickam
% Developed in collaboration with J. Gilbert and E. Ng.

ccolamd_make
addpath (pwd)
fprintf ('\nThe following path has been added.  You may wish to add it\n') ;
fprintf ('permanently, using the MATLAB pathtool command.\n') ;
fprintf ('%s\n\n', pwd) ;
ccolamd_demo
