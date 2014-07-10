function pos_ECEF = LatLonHRad_to_ECEF(LatLonH)
% convert latitude, longitude, height to XYZ in ECEF coordinates 
% LatLonH(1) : latitude in radian 
% LatLonH(2) : longitude in radian
% LatLonH(3) : height in meter
%
% Source: A. Chatfield, "Fundamentals of High Accuracy Inertial
% Navigation", 1997. pp. 10 Eq 1.18
%

% constants
a = 6378137.0; %m
e_sqr =0.006694379990141317; 
% b = 6356752.3142; % m 

%RAD_PER_DEG = pi/180;
phi = LatLonH(1);%*RAD_PER_DEG;
lambda = LatLonH(2);%*RAD_PER_DEG;
h = LatLonH(3);

RN = a/sqrt(1 - e_sqr*sin(phi)^2);

pos_ECEF = zeros(3,1);
pos_ECEF(1) = (RN + h )*cos(phi)*cos(lambda);
pos_ECEF(2) = (RN + h )*cos(phi)*sin(lambda);
pos_ECEF(3) = (RN*(1-e_sqr) + h)*sin(phi) ;