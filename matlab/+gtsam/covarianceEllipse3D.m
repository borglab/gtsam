function sc = covarianceEllipse3D(c,P,scale)
% covarianceEllipse3D plots a Gaussian as an uncertainty ellipse
% Based on Maybeck Vol 1, page 366
% k=2.296 corresponds to 1 std, 68.26% of all probability
% k=11.82 corresponds to 3 std, 99.74% of all probability
%
% Modified from http://www.mathworks.com/matlabcentral/newsreader/view_thread/42966

hold on

[e,s] = svd(P);
k = 11.82; 
radii = k*sqrt(diag(s));

if exist('scale', 'var')
    radii = radii * scale;
end

% generate data for "unrotated" ellipsoid
[xc,yc,zc] = ellipsoid(0,0,0,radii(1),radii(2),radii(3),8);

% rotate data with orientation matrix U and center M
data = kron(e(:,1),xc) + kron(e(:,2),yc) + kron(e(:,3),zc);
n = size(data,2);
x = data(1:n,:)+c(1);
y = data(n+1:2*n,:)+c(2);
z = data(2*n+1:end,:)+c(3);

% now plot the rotated ellipse
sc = surf(x,y,z,abs(xc));
shading interp
alpha(0.5)
axis equal