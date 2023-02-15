function [x,y,z] = created3DgaussianEllipsoid(mu,V,D, scale)
% For N standard deviations spread of data, the radii of the eliipsoid will
% be given by N*SQRT(eigenvalues).

radii = scale*diag(D);
% generate data for "unrotated" ellipsoid
[xc,yc,zc] = ellipsoid(0,0,0,radii(1),radii(2),radii(3));

% rotate data with orientation matrix U and center mu
I = V;
% I = eye(size(V));

a = kron(I(:,1),xc); 
b = kron(I(:,2),yc); 
c = kron(I(:,3),zc);

data = a+b+c; n = size(data,2);

x = data(1:n,:)+mu(1); 
y = data(n+1:2*n,:)+mu(2); 
z = data(2*n+1:end,:)+mu(3);

end