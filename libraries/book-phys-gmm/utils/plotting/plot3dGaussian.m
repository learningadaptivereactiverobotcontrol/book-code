function handles = plot3dGaussian(Priors, Mus,Sigmas )
%
%  Function to plot a 3D Gaussian Mixture Model
%
%   input -----------------------------------------------------------------
%
%     o Priors : (1 x K),       weights, K is the number of Gaussian functions
%
%     o Mu     : (D x K),       Means
%
%     o Sigma  : (D x D x K),   Covariances
%
%   output ----------------------------------------------------------------
%
%
%


K = size(Mus,2);
npts = 500; 

handles = zeros(K,1);

weights = rescale(Priors,min(Priors),max(Priors),0.01,0.8);

weights(isnan(weights)==1) = 0.8;

for k = 1:K    
    [x,y,z] = sphere(npts);
    ap = [x(:) y(:) z(:)]';
    [v,d]=eig(Sigmas(:,:,k));
    if any(d(:) < 0)
        fprintf('warning: negative eigenvalues\n');
        d = max(d,0);
    end
    d =  sqrt(d); % convert variance to sdwidth*sd
    bp = (v*d*ap) + repmat(Mus(:,k), 1, size(ap,2));
    xp = reshape(bp(1,:), size(x));
    yp = reshape(bp(2,:), size(y));
    zp = reshape(bp(3,:), size(z));
   % handles(k) = surf(xp,yp,zp);
    handles(k) = mesh(xp(1:5:end,1:5:end),yp(1:5:end,1:5:end),zp(1:5:end,1:5:end),'edgecolor', 'k','facecolor','none');
    
    scatter3(Mus(1,k),Mus(2,k),Mus(3,k),10,'filled','ko');
    %sh = surfl(xp, yp, zp);
    %set(sh,'FaceColor',[0.5,0.5,0.5],'FaceAlpha',weights(k),'EdgeColor','none')



end

end

