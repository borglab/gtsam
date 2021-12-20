function plotCamera(pose, axisLength)
	hold on

    C = pose.translation();
    R = pose.rotation().matrix();
    
    xAxis = C+R(:,1)*axisLength;
    L = [C xAxis]';
    h_x = line(L(:,1),L(:,2),L(:,3),'Color','r');
    
    yAxis = C+R(:,2)*axisLength;
    L = [C yAxis]';
    h_y = line(L(:,1),L(:,2),L(:,3),'Color','g');
    
    zAxis = C+R(:,3)*axisLength;
    L = [C zAxis]';
    h_z = line(L(:,1),L(:,2),L(:,3),'Color','b');
    
    axis equal
end
