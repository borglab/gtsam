% Christian Potthast
% simulate measurements

function [measurements,odometry] = simulate_measurements(map, pose, visibility, noise_sigma, odo_sigma)
m = size(pose,2);
n = size(map,2);
measurements = {};
odometry = {};
k =1;
for i = 1:m
    js = find(visibility(1:n,i+n));
    if size(js ,1) > 0
        for j = js'
            z = map(:,j)-pose(:,i)+randn(2,1)*noise_sigma;
            measurement = struct('z',z,'i',i,'j',j);
            measurements{k}=measurement;
            k = k+1;
        end
    end
    if i>1
        odometry{i}= pose(:,i)-pose(:,i-1)+randn(2,1)*odo_sigma;
    end
end