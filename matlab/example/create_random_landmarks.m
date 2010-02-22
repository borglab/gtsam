% Christian Potthast
% Create a map with random landmarks

function map = create_random_landmarks(numberOfLandmarks, mappingArea)

points = rand(2,numberOfLandmarks);

map=[points(1,:)*mappingArea(1);points(2,:)*mappingArea(2)];
