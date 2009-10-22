% Christian Potthast
% create a configuration

function config = create_config(n,m)

config = VectorConfig();

for j = 1:n
    config.insert(sprintf('m%d',j), [0;0]);
end

for i = 1:m
    config.insert(sprintf('x%d',i), [0;0]); 
end