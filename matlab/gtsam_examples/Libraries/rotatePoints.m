function rotatedData = rotatePoints(alignmentVector, originalData)

% rotatedData = rotatePoints(alignmentVector, originalData) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     Rotate the 'originalData' in the form of Nx2 or Nx3 about the origin by aligning the x-axis with the alignment vector
% 
%       Rdata = rotatePoints([1,2,-1], [Xpts(:), Ypts(:), Zpts(:)]) - rotate the (X,Y,Z)pts in 3D with respect to the vector [1,2,-1]
% 
%       Rotating using spherical components can be done by first converting using [dX,dY,dZ] = cart2sph(theta, phi, rho);  alignmentVector = [dX,dY,dZ];
% 
% Example:
%   %% Rotate the point [3,4,-7] with respect to the following:
%   %%%% Original associated vector is always [1,0,0]
%   %%%% Calculate the appropriate rotation requested with respect to the x-axis.  For example, if only a rotation about the z-axis is
%   %%%% sought, alignmentVector = [2,1,0] %% Note that the z-component is zero
%   rotData = rotatePoints(alignmentVector, [3,4,-7]);
% 
%     Author: Shawn Arseneau
%     Created: Feb.2, 2006
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    alignmentDim = numel(alignmentVector);
    DOF = size(originalData,2); %---- DOF = Degrees of Freedom (i.e. 2 for two dimensional and 3 for three dimensional data)
    
    if alignmentDim~=DOF    
        error('Alignment vector does not agree with originalData dimensions');      
    end
    if DOF<2 || DOF>3      
        error('rotatePoints only does rotation in two or three dimensions');        
    end
    
        
    if DOF==2  % 2D rotation...        
        [rad_theta, rho] = cart2pol(alignmentVector(1), alignmentVector(2));    
        deg_theta = -1 * rad_theta * (180/pi);
        ctheta = cosd(deg_theta);  stheta = sind(deg_theta);
        
        Rmatrix = [ctheta, -1.*stheta;...
                   stheta,     ctheta];
        rotatedData = originalData*Rmatrix;        
        
    else    % 3D rotation...        
        [rad_theta, rad_phi, rho] = cart2sph(alignmentVector(1), alignmentVector(2), alignmentVector(3));
        rad_theta = rad_theta * -1; 
        deg_theta = rad_theta * (180/pi);
        deg_phi = rad_phi * (180/pi); 
        ctheta = cosd(deg_theta);  stheta = sind(deg_theta);
        Rz = [ctheta,   -1.*stheta,     0;...
              stheta,       ctheta,     0;...
              0,                 0,     1];                  %% First rotate as per theta around the Z axis
        rotatedData = originalData*Rz;

        [rotX, rotY, rotZ] = sph2cart(-1* (rad_theta+(pi/2)), 0, 1);          %% Second rotation corresponding to phi
        rotationAxis = [rotX, rotY, rotZ];
        u = rotationAxis(:)/norm(rotationAxis);        %% Code extract from rotate.m from MATLAB
        cosPhi = cosd(deg_phi);
        sinPhi = sind(deg_phi);
        invCosPhi = 1 - cosPhi;
        x = u(1);
        y = u(2);
        z = u(3);
        Rmatrix = [cosPhi+x^2*invCosPhi        x*y*invCosPhi-z*sinPhi     x*z*invCosPhi+y*sinPhi; ...
                   x*y*invCosPhi+z*sinPhi      cosPhi+y^2*invCosPhi       y*z*invCosPhi-x*sinPhi; ...
                   x*z*invCosPhi-y*sinPhi      y*z*invCosPhi+x*sinPhi     cosPhi+z^2*invCosPhi]';

        rotatedData = rotatedData*Rmatrix;        
    end














