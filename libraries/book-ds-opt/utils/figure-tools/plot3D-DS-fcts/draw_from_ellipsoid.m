function pnts = draw_from_ellipsoid( covmat, cent, npts )

% function pnts = draw_from_ellipsoid( covmat, cent, npts )
%
% This function draws points uniformly from an n-dimensional ellipsoid
% with edges and orientation defined by the the covariance matrix covmat. 
% The number of points produced in the n-dimensional space is given by 
% npts, with an output array of npts by ndims. The centre of each dimension
% is given in the vector cent.

% get number of dimensions of the covariance matrix
ndims = length(covmat);

% calculate eigenvalues and vectors of the covariance matrix
[v, e] = eig(covmat);

% check size of cent and transpose if necessary
sc = size(cent);
if sc(1) > 1
    cent = cent';
end 

% generate radii of hyperspheres
rs = rand(npts,1);

% generate points
pt = randn(npts,ndims);

% get scalings for each point onto the surface of a unit hypersphere
fac = sum(pt(:,:)'.^2);

% calculate scaling for each point to be within the unit hypersphere 
% with radii rs
fac = (rs.^(1/ndims)) ./ sqrt(fac');

pnts = zeros(npts,ndims);

% scale points to the ellipsoid using the eigenvalues and rotate with 
% the eigenvectors and add centroid
d = sqrt(diag(e));
for i=1:npts
    % scale points to a uniform distribution within unit hypersphere
    pnts(i,:) = fac(i)*pt(i,:);
    
    % scale and rotate to ellipsoid
    pnts(i,:) = (pnts(i,:) .* d' * v') + cent;
end

end