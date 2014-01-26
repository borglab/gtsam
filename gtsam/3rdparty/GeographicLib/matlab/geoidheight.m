function geoidheight(~, ~, ~)
%geoidheight  Compute geoid height
%
%   CAUTION: THIS MAY CAUSE MATLAB TO CRASH!!  This occurs when the interface
%   is compiled with Visual Studio 2008.
%
%   [height, gradient] = geoidheight(latlong)
%   [height, gradient] = geoidheight(latlong, geoidname)
%   [height, gradient] = geoidheight(latlong, geoidname, geoiddir)
%
%   latlong is an M x 2 matrix
%       latitude = latlong(:,1) in degrees
%       longitude = latlong(:,2) in degrees
%   geoidname is the name of the geoid; choices are (default egm96-5)
%       egm84-30  egm84-15
%       egm96-15  egm96-5
%       egm2008-5 egm2008-2_5 egm2008-1
%   geoiddir is the directory containing the geoid models (default empty
%       string meaning system default)
%
%   height is an M x 1 matrix
%       geoidheight = height(:,1) height of geoid in meters
%   gradient is the gradient of the geoid height
%       gradn = gradient(:,1) gradient of height in northerly direction
%       grade = gradient(:,2) gradient of height in easterly direction
%
% This is an interface to the GeographicLib C++ routine
%     Geoid::operator()
% See the documentation on this function for more information:
% http://geographiclib.sf.net/html/classGeographicLib_1_1Geoid.html
  error('Error: executing .m file instead of compiled routine');
end
% geoidheight.m
% Matlab .m file for looking up geoid heights
%
% Copyright (c) Charles Karney (2010-2011) <charles@karney.com> and licensed
% under the MIT/X11 License.  For more information, see
% http://geographiclib.sourceforge.net/
