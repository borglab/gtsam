function A = choleskyNaive(A)
%ELIMINATEPP Gaussian elimination with partial pivoting.  Eliminates a
%matrix A (sparse or full) into an upper-triangular factor R.

% Loop over rows of A
q = min(size(A,2), size(A,1));
lastA = 0;
for i = 1:q
    % Check for valid pivot
    if A(i,i) <= 0.0
        A = [];
        return;
    end
    
    if lastA == A(i,i)
        1;
    end
    lastA = A(i,i);
    
    % Scale row to be the square-root factor
    A(i,i) = sqrt(A(i,i));
    A(i,i+1:end) = A(i,i+1:end) ./ A(i,i);
    
    % Apply low-rank update to remanining lower-right submatrix
    A(i+1:end, i+1:end) = A(i+1:end, i+1:end) - ...
       A(i, i+1:end)' * A(i, i+1:end);
    
    % Zero-out the column below the current row
    A(i+1:end, i) = 0;
end

end

