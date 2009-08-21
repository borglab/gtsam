% Christian Potthast
% create an elimination ordering

function ord = create_ordering(n,m)

ord = Ordering();

for j = 1:n
    ord.push_back(sprintf('m%d',j));
end

for i = 1:m
    ord.push_back(sprintf('x%d',i)); 
end