function [graph,initial] = load3D(filename,model,successive,N)
% load3D: read TORO 3D pose graph
% cannot read noise model from file yet, uses specified model
% if [successive] is tru, constructs initial estimate from odometry

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
graph = pose3SLAMGraph;
initial = pose3SLAMValues;
origin=gtsamPose3;
initial.insertPose(0,origin);
n=size(lines,1);
if nargin<4, N=n;end

for i=1:n
    line_i=lines{i};
    if strcmp('VERTEX3',line_i(1:7))
        v = textscan(line_i,'%s %d %f %f %f %f %f %f',1);
        i=v{2};
        if (~successive & i<N | successive & i==0)
            t = gtsamPoint3(v{3}, v{4}, v{5});
            R = gtsamRot3_ypr(v{8}, -v{7}, v{6});
            initial.insertPose(i, gtsamPose3(R,t));
        end
    elseif strcmp('EDGE3',line_i(1:5))
        e = textscan(line_i,'%s %d %d %f %f %f %f %f %f',1);
        i1=e{2};
        i2=e{3};
        if i1<N & i2<N
            if ~successive | abs(i2-i1)==1
                t = gtsamPoint3(e{4}, e{5}, e{6});
                R = gtsamRot3_ypr(e{9}, e{8}, e{7});
                dpose = gtsamPose3(R,t);
                graph.addRelativePose(i1, i2, dpose, model);
                if successive
                    if i2>i1
                        initial.insertPose(i2,initial.pose(i1).compose(dpose));
                    else
                        initial.insertPose(i1,initial.pose(i2).compose(dpose.inverse));
                    end
                end
            end
        end
    end
end

