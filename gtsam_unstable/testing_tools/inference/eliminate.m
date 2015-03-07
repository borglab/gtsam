function [ RS, J ] = eliminate(Ab, js)
%ELIMINATE This is a simple scalar-level function for eliminating the
%matrix Ab into a conditional part RS and a marginal part J.  The scalars
%in js are the ones eliminated (corresponding to columns of Ab).  Note that
%the returned matrices RS and J will both be the same width as Ab and will
%contain columns of all zeros - that is, the column indices will stay the
%same in all matrices, unlike in GTSAM where matrix data is only stored for
%involved variables.

if size(Ab,1) < size(Ab,2)-1
    Ab = [ Ab; zeros(size(Ab,2)-1-size(Ab,1), size(Ab,2)) ];
end

% Permute columns
origCols = 1:size(Ab,2);
jsFront = js;
jsBack = setdiff(origCols, js);
jsPermuted = [ jsFront jsBack ];
Abpermuted = Ab(:,jsPermuted);

% Eliminate (this sparse stuff prevents qr from introducing a permutation
R = full(qr(sparse(Abpermuted)));

% Find row split
firstRowOfMarginal = numel(jsFront) + 1;
for i = size(R,1) : -1 : 1
    firstNnz = find(R(i,:), 1, 'first');
    if ~isempty(firstNnz)
        if firstNnz > numel(jsFront)
            firstRowOfMarginal = i;
        else
            break;
        end
    end
end

% Undo permutation
Runpermuted(:,jsPermuted) = R;

% Split up
RS = Runpermuted(1:firstRowOfMarginal-1, :);
J = Runpermuted(firstRowOfMarginal:size(Ab,2)-1, :);

end

