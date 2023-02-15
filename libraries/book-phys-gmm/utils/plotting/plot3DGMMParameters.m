function [hf] = plot3DGMMParameters(Xi_ref, GMM, labels)    
    hf = figure('Color',[1 1 1]); hold on;grid on;
    
    % GMM 
    Priors    = GMM.Priors;
    Mu        = GMM.Mu;
    Sigma     = GMM.Sigma;    
    K         = length(unique(labels));
    
    % Clustered Sigmas GMM
    colors = vivid(K);
    colors = hsv(K);
    colors = jet(K);
    
    for k=1:K        
        scatter3(Xi_ref(1,labels==k), Xi_ref(2,labels==k), Xi_ref(3,labels==k), 10, 'MarkerEdgeColor','k','MarkerFaceColor',colors(k,:)); hold on;        
        [V,D]=eig(Sigma(:,:,k));
        scale = 5;
        [x,y,z] = created3DgaussianEllipsoid(Mu(:,k),V,D, scale);

        % This makes the ellipsoids beautiful  
        surf(x, y, z,'FaceColor',colors(k,:),'FaceAlpha', 0.25, 'FaceLighting','phong','EdgeColor','none');        
        camlight
    end            

    grid on;
    xlabel('$\xi_1$', 'Interpreter', 'LaTex', 'FontSize',15);
    ylabel('$\xi_2$', 'Interpreter', 'LaTex','FontSize',15);
    zlabel('$\xi_3$', 'Interpreter', 'LaTex','FontSize',15);
    set(gca,'FontSize',16);
end