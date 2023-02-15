function [hf] = plot3DGMM(GMM, labels, Xi_ref)    
    hf = figure('Color',[1 1 1]); hold on;grid on;
    
    % GMM 
    Priors    = GMM.Priors;
    Mu        = GMM.Mu;
    Sigma     = GMM.Sigma;    
    handles   = plot3dGaussian(Priors, Mu,Sigma );
    alpha     = rescale(Priors,min(Priors),max(Priors),0.1,0.8);    
    K          = length(unique(labels));
    
    % Clustered Sigmas GMM
%     colors = hsv(length(unique(labels)));
    colors = vivid();
    
    for k=1:K
        scatter3(Xi_ref(1,:),Xi_ref(1,:),Xi_ref(1,:))
    end
    
    
    for i=1:size(handles,1)
        set(handles(i),'FaceLighting','phong','FaceColor',colors(labels(i),:),'FaceAlpha',alpha(i),'AmbientStrength',0.1,'EdgeColor','none');
    end    
    camlight
    grid on;
    title('Estimated Gaussian Mixture Model Parameters', 'Interpreter','LaTex');
    set(gca,'FontSize',16);
end