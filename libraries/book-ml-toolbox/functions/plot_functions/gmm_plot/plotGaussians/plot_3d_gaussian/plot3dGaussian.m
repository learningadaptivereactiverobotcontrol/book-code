function handles = plot3dGaussian(Priors, Mus,Sigmas )
% 

K = size(Mus,2);
npts = 50; 

handles = zeros(K,1);
% weights = Priors;
weights = 1/length(Priors)*ones(length(Priors));

% weights = rescale(Priors,min(Priors),max(Priors),0.01,0.8);

% weights(isnan(weights)==1) = 0.8;
K_colors = hsv(K);
for k = 1:K    
    [x,y,z] = sphere(npts);
    ap = [x(:) y(:) z(:)]';
    [v,d]=eig(Sigmas(:,:,k));
    if any(d(:) < 0)
        fprintf('warning: negative eigenvalues\n');
        d = max(d,0);
    end
    
    if eig(Sigmas(:,:,k)) < 1e-3
        d = d*100;
    end
    
    d =  sqrt(d); % convert variance to sdwidth*sd
    bp = (v*d*ap) + repmat(Mus(:,k), 1, size(ap,2));
    xp = reshape(bp(1,:), size(x));
    yp = reshape(bp(2,:), size(y));
    zp = reshape(bp(3,:), size(z));
    handles(k) = surf(xp,yp,zp);

    
%     [~,D] = eig(Sigmas(:,:,k));
%     eigV = sqrt(diag(D))
%     [x, y, z] = ellipsoid(Mus(1,k),Mus(2,k),Mus(3,k),eigV(1,1),eigV(2,1),eigV(3,1),30);

    scatter3(Mus(1,k),Mus(2,k),Mus(3,k),2,'filled','ko');
%     sh = surfl(xp, yp, zp);
    alpha 0.05
%     set(sh,'FaceColor',K_colors(k,:),'EdgeColor','none')


end

end

