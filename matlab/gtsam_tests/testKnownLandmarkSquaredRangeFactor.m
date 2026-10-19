%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Tests MATLAB wrapper dimension validation for
% KnownLandmarkSquaredRangeFactor.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function testKnownLandmarkSquaredRangeFactor()
model = gtsam.noiseModel.Unit.Create(1);
key = 3;

% Both specializations accept correctly sized landmark vectors.
factor2 = gtsam.KnownLandmarkSquaredRangeFactor2(key, zeros(2, 1), 5.0, model);
factor3 = gtsam.KnownLandmarkSquaredRangeFactor3(key, zeros(3, 1), 5.0, model);
assert(~isempty(factor2));
assert(~isempty(factor3));

% Both specializations reject undersized and oversized landmark vectors.
for landmarkSize = [1, 3]
    assertInvalidLandmarkDimension(@() gtsam.KnownLandmarkSquaredRangeFactor2( ...
        key, zeros(landmarkSize, 1), 5.0, model));
end

for landmarkSize = [2, 4]
    assertInvalidLandmarkDimension(@() gtsam.KnownLandmarkSquaredRangeFactor3( ...
        key, zeros(landmarkSize, 1), 5.0, model));
end
end

function assertInvalidLandmarkDimension(constructor)
% Ensure an invalid landmark dimension produces a catchable MATLAB error.
try
    constructor();
catch ME
    assert(contains(ME.message, 'invalid landmark dimension'), ...
        'Expected a landmark dimension error; received: %s', ME.message);
    return;
end
error('Expected an invalid landmark dimension error, but none was thrown.');
end
