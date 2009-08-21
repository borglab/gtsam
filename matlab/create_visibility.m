% Christian Potthast
% create visibility matrix

function V = create_visibility(map,pose,threshold)

n = size(map,2);
m = size(pose,2);
V = sparse([],[],[],n+m,n+m);

for t = 1:m
    % find measurements within Manhattan range
    js = find(2==sum(abs(map-pose(:,t)*ones(1,n))<[threshold;threshold]*ones(1,n)));
    for j = js
        V(j,t+n)=1;
    end
    % add in odometry links
    if t>1
       V((t+n)-1,t+n)=1; 
    end
end