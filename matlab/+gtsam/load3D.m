function [graph,initial] = load3D(filename,model,successive,N)
% load3D reads a TORO-style 3D pose graph
% cannot read noise model from file yet, uses specified model
% if [successive] is tru, constructs initial estimate from odometry

import gtsam.*

if nargin<3, successive=false; end
fid = fopen(filename);
if fid < 0
    error(['load2D: Cannot open file ' filename]);
end

% scan all lines into a cell array
columns=textscan(fid,'%s','delimiter','\n');
fclose(fid);
lines=columns{1};

% loop over lines and add vertices
graph = NonlinearFactorGraph;
initial = Values;
origin=gtsam.Pose3;
initial.insert(0,origin);
n=size(lines,1);
if nargin<4, N=n;end

for i=1:n
    line_i=lines{i};
    if strcmp('VERTEX3',line_i(1:7))
        v = textscan(line_i,'%s %d %f %f %f %f %f %f',1);
        i1=v{2};
        if (~successive && i1<N || successive && i1==0)
            t = gtsam.Point3(v{3}, v{4}, v{5});
            R = gtsam.Rot3.Ypr(v{8}, -v{7}, v{6});
            initial.insert(i1, gtsam.Pose3(R,t));
        end
    elseif strcmp('EDGE3',line_i(1:5))
        e = textscan(line_i,'%s %d %d %f %f %f %f %f %f',1);
        i1=e{2};
        i2=e{3};
        if i1<N && i2<N
            if ~successive || abs(i2-i1)==1
                t = gtsam.Point3(e{4}, e{5}, e{6});
                R = gtsam.Rot3.Ypr(e{9}, e{8}, e{7});
                dpose = gtsam.Pose3(R,t);
                graph.add(BetweenFactorPose3(i1, i2, dpose, model));
                if successive
                    if i2>i1
                        initial.insert(i2,initial.atPose3(i1).compose(dpose));
                    else
                        initial.insert(i1,initial.atPose3(i2).compose(dpose.inverse));
                    end
                end
            end
        end
    end
end

