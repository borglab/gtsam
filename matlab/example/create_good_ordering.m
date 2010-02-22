% Christian Potthast
% create an elimination ordering

function ord = create_good_ordering(n,m,measurements)

ord = Ordering();
j=1;
pose=1;
mes=size(measurements,2);
while (pose<=m)&&(j<=mes)
    ord.push_back(sprintf('x%d',pose));
    while (j<n)&&(measurements{j}.i==pose)
        ord.push_back(sprintf('l%d',j));
        j=j+1;
    end
    pose=pose+1;
end