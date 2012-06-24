function plot3DTrajectory(values,style,frames,scale)
% plot3DTrajectory
if nargin<3,frames=false;end
if nargin<4,scale=0;end

T=values.translations()
plot3(T(:,1),T(:,2),T(:,3),style); hold on
if frames
    N=values.size;
    for i=0:N-1
        pose = values.pose(i);
        t = pose.translation;
        R = pose.rotation.matrix;
        quiver3(t.x,t.y,t.z,R(1,1),R(2,1),R(3,1),scale,'r');
        quiver3(t.x,t.y,t.z,R(1,2),R(2,2),R(3,2),scale,'g');
        quiver3(t.x,t.y,t.z,R(1,3),R(2,3),R(3,3),scale,'b');
    end
end