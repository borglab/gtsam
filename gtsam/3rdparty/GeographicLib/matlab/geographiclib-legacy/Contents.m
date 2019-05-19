% GeographicLib legacy functions
% Version 1.42 2015-04-27
%
%   The functions in this directory are DEPRECATED.  They are wrapper
%   routines for the GeographicLib toolbox, which is available at
%
%     href="http://www.mathworks.com/matlabcentral/fileexchange/50605
%
%   Directly using the GeographicLib toolbox provides greater functionality
%   and flexiblility.
%
% Geodesics
%   geodesicdirect         - Wrapper for geodreckon
%   geodesicinverse        - Wrapper for geoddistance
%   geodesicline           - Another wrapper for geodreckon
%   polygonarea            - Wrapper for geodarea
%
% Grid systems
%   utmupsforward          - Wrapper for utmups_fwd
%   utmupsreverse          - Wrapper for utmups_inv
%   mgrsforward            - Wrapper for mgrs_fwd
%   mgrsreverse            - Wrapper for mgrs_inv
%
% Geoid lookup
%   geoidheight            - Wrapper for geoid_height
%
% Geometric transformations
%   geocentricforward      - Wrapper for geocent_fwd
%   geocentricreverse      - Wrapper for geocent_inv
%   localcartesianforward  - Wrapper for loccart_fwd
%   localcartesianreverse  - Wrapper for loccart_inv

% Copyright (c) Charles Karney (2015) <charles@karney.com>.
