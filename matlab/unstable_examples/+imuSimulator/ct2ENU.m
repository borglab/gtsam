function [dx,dy,dz]=ct2ENU(dX,dY,dZ,lat,lon)
% CT2LG  Converts CT coordinate differences to local geodetic.
%   Local origin at lat,lon,h. If lat,lon are vectors, dx,dy,dz
%   are referenced to orgin at lat,lon of same index. If
%   astronomic lat,lon input, output is in local astronomic
%   system. Vectorized in both dx,dy,dz and lat,lon. See also
%   LG2CT.
% Version: 2011-02-19
% Useage:  [dx,dy,dz]=ct2lg(dX,dY,dZ,lat,lon)
% Input:   dX  - vector of X coordinate differences in CT
%          dY  - vector of Y coordinate differences in CT
%          dZ  - vector of Z coordinate differences in CT
%          lat - lat(s) of local system origin (rad); may be vector
%          lon - lon(s) of local system origin (rad); may be vector
% Output:  dx  - vector of x coordinates in local system (east)
%          dy  - vector of y coordinates in local system (north)
%          dz  - vector of z coordinates in local system (up)

% Copyright (c) 2011, Michael R. Craymer
% All rights reserved.
% Email: mike@craymer.com

if nargin ~= 5
  warning('Incorrect number of input arguements');
  return
end

n=length(dX);
if length(lat)==1
  lat=ones(n,1)*lat;
  lon=ones(n,1)*lon;
end
R=zeros(3,3,n);

R(1,1,:)=-sin(lat').*cos(lon');
R(1,2,:)=-sin(lat').*sin(lon');
R(1,3,:)=cos(lat');
R(2,1,:)=sin(lon');
R(2,2,:)=-cos(lon');
R(2,3,:)=zeros(1,n);
R(3,1,:)=cos(lat').*cos(lon');
R(3,2,:)=cos(lat').*sin(lon');
R(3,3,:)=sin(lat');

RR=reshape(R(1,:,:),3,n);
dx_temp=sum(RR'.*[dX dY dZ],2);
RR=reshape(R(2,:,:),3,n);
dy_temp=sum(RR'.*[dX dY dZ],2);
RR=reshape(R(3,:,:),3,n);
dz=sum(RR'.*[dX dY dZ],2);

dx = -dy_temp;
dy = dx_temp;


