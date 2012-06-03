function plotTrajectory(values,color)
% plotTrajectory: plot the poses in a values object
% assumes keys are 0 to N-1, where N is values.size()
X=[];Y=[];
for i=0:values.size()-1
    pose_i = values.pose(i);
    X=[X;pose_i.x];
    Y=[Y;pose_i.y];
end
plot(X,Y,color);
