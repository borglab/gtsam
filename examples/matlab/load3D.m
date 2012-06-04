function [graph,initial] = load3D(filename,model,N)
% load3D: read TORO 3D pose graph
% cannot read noise model from file yet, uses specified model
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
n=size(lines,1);
if nargin<3, N=n;end

for i=1:n
    line_i=lines{i};
    if strcmp('VERTEX3',line_i(1:7))
        v = textscan(line_i,'%s %d %f %f %f %f %f %f',1);
        if v{2}<N
            t = gtsamPoint3(v{3}, v{4}, v{5});
            R = gtsamRot3_ypr(v{6}, v{7}, v{8});
            initial.insertPose(v{2}, gtsamPose3(R,t));
        end
    elseif strcmp('EDGE3',line_i(1:5))
        e = textscan(line_i,'%s %d %d %f %f %f %f %f %f',1);
        if e{2}<N & e{3}<N
            t = gtsamPoint3(e{4}, e{5}, e{6});
            R = gtsamRot3_ypr(e{7}, e{8}, e{9});
            graph.addConstraint(e{2}, e{3}, gtsamPose3(R,t), model);
        end
    end
end

