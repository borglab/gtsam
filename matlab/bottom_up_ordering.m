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
[~, node_order] = sort(node_levels, 'descend');  % the order of the nodes in leaves-to-root order

ordering = Ordering();
for i = 1:length(node_order)
    ordering.push_back(sprintf('x%d', node_order(i)));
end
end