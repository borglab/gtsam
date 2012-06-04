function plot3DTrajectory(values,style,frames)
% plot3DTrajectory
if nargin<3,frames=false;end

plot3(values.xs(),values.ys(),values.zs(),style); hold on
if frames
    for i=0:N-1
        pose = values.pose(i);
        t = pose.translation;
        R = pose.rotation.matrix;
        quiver3(t.x,t.y,t.z,R(1,1),R(2,1),R(3,1),'r');
        quiver3(t.x,t.y,t.z,R(1,2),R(2,2),R(3,2),'g');
        quiver3(t.x,t.y,t.z,R(1,3),R(2,3),R(3,3),'b');
    end
end