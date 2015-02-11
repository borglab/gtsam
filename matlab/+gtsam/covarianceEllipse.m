function h = covarianceEllipse(x,P,color, k)
% covarianceEllipse plots a Gaussian as an uncertainty ellipse
% Based on Maybeck Vol 1, page 366
% k=2.296 corresponds to 1 std, 68.26% of all probability
% k=11.82 corresponds to 3 std, 99.74% of all probability
%
% covarianceEllipse(x,P,color,k)
% it is assumed x and y are the first two components of state x
% k is scaling for std deviations, defaults to 1 std

hold on

[e,s] = eig(P(1:2,1:2));
s1 = s(1,1);
s2 = s(2,2);
if nargin<4, k = 2.296; end;
[ex,ey] = ellipse( sqrt(s1*k)*e(:,1), sqrt(s2*k)*e(:,2), x(1:2) );
h = line(ex,ey,'color',color);

    function [x,y] = ellipse(a,b,c);
        % ellipse: return the x and y coordinates for an ellipse
        % [x,y] = ellipse(a,b,c);
        % a, and b are the axes. c is the center
        
        global ellipse_x ellipse_y
        if ~exist('elipse_x')
            q =0:2*pi/25:2*pi;
            ellipse_x = cos(q);
            ellipse_y = sin(q);
        end
        
        points = a*ellipse_x + b*ellipse_y;
        x = c(1) + points(1,:);
        y = c(2) + points(2,:);
    end

end
