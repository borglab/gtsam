% Christian Potthast
% plot a configuration 

function plot_config(config,n,m)

hold on;

for j = 1:n
    key = sprintf('m%d',j);
    mj = config.get(key);
    plot(mj(1), mj(2),'r*');
end

for i = 1:m
    key = sprintf('x%d',i);
    xi = config.get(key);
    plot(xi(1), xi(2),'rx');
end

