function [] = plot_trajectory(pose, step, color, plot_name,a,b,c)
% Plot a collection of poses: positions are connected by a solid line 
% of color "color" and it is shown a ref frame for each pose. 
% Plot_name defines the name of the created figure.

x = getxyz(pose,1); 
y = getxyz(pose,2); 
z = getxyz(pose,3); 
plot3(x,y,z,color)
n = length(x)-1;
hold on
for i=1:step:n+1
    ref_frame_plot(pose(i).R,pose(i).p,a,b,c) 
end
title(plot_name);
xlabel('x')
ylabel('y')
zlabel('z')
view(3)  