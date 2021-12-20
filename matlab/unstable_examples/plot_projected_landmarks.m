function [] = plot_projected_landmarks( a, landmarks, measurements )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

persistent h;

if ishghandle(h)
    delete(h);
end

measurement_keys = gtsam.KeyVector(measurements.keys);
nrMeasurements = measurement_keys.size;

if nrMeasurements == 0
    return;
end

x = zeros(1,nrMeasurements);
y = zeros(1,nrMeasurements);
z = zeros(1,nrMeasurements);

% Plot points and covariance matrices
for i = 0:measurement_keys.size-1
    key = measurement_keys.at(i);
    key_index = gtsam.symbolIndex(key);
    p = landmarks.atPoint3(gtsam.symbol('l',key_index));
    
    x(i+1) = p(1);
    y(i+1) = p(2);
    z(i+1) = p(3);
    
end

h = plot3(a, x,y,z,'rd', 'LineWidth',3);

end

