function plotCamera(pose, axisLength)
    C = pose.translation().vector();
    R = pose.rotation().matrix();
    
    xAxis = C+R(:,1)*axisLength;
    L = [C xAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','r');
    
    yAxis = C+R(:,2)*axisLength;
    L = [C yAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','g');
    
    zAxis = C+R(:,3)*axisLength;
    L = [C zAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','b');
    
    axis equal
end