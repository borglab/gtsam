% find a bottom to up ordering given the tree structure {pred} returned by matlab's graphminspantree
function [ordering] = bottom_up_ordering(pred)

%% compute the levels of the nodes
parents = [0];
node_levels = zeros(length(pred), 1);
current_level = 1;
while ~isempty(parents)
    parents = find(ismember(pred, parents));
    node_levels(parents) = current_level;
    current_level = current_level + 1;
end
max_level = current_level - 1;

ordering = Ordering();
for level = max_level:-1:1
    ids = find(node_levels==level);
    for i = 1:length(ids)
        ordering.push_back(sprintf('x%d', ids(i)));
    end
end
end