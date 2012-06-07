function [graph,initial] = load2D(filename,model)
% load2D: read TORO pose graph
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
graph = pose2SLAMGraph;
initial = pose2SLAMValues;
n=size(lines,1);
for i=1:n
    line_i=lines{i};
    if strcmp('VERTEX2',line_i(1:7))
        v = textscan(line_i,'%s %d %f %f %f',1);
        initial.insertPose(v{2}, gtsamPose2(v{3}, v{4}, v{5}));
    elseif strcmp('EDGE2',line_i(1:5))
        e = textscan(line_i,'%s %d %d %f %f %f',1);
        graph.addOdometry(e{2}, e{3}, gtsamPose2(e{4}, e{5}, e{6}), model);
    end
end

