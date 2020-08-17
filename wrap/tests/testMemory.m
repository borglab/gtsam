%MATLAB testing file for memory allocation and leaks
%Andrew Melim

addpath([pwd,'/../../../toolbox/gtsam']);
for i=1:100000
    p = gtsamPoint2()
end
