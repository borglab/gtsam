% Christian Potthast
% Create a map with random landmarks

function map = create_random_landmarks(visibilityTh, mappingArea, steps)
map=[];
points1=1:visibilityTh:mappingArea(1);
points2=1:visibilityTh:mappingArea(2);
 for i=1:size(points1,2)
  map=[map,[points1(1,i)-steps;points2(1,i)],[points1(1,i);points2(1,i)-steps]];
 end

% for i=1:size(points1,2)
%     for j=1:size(points2,2)
%     map=[map,[points1(1,i);points2(1,j)]];
%     end
% end
