function [c] = getxyz(poses, j)
% The function extract the Cartesian variables from pose (pose.p = positions, 
% pose.R = rotations). In particular, if there are T poses, 
% - getxyz(pose, 1) estracts the vector x \in R^T, 
% - getxyz(pose, 2) estracts the vector y \in R^T,
% - getxyz(pose, 3) estracts the vector z \in R^T.

L = length(poses);
c = [];
for i=1:L % for each pose
    c = [c poses(i).p(j)];
end

c = c(:); % column vector