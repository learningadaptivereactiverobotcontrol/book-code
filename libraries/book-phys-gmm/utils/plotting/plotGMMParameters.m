function [ handle, h_gmm, h_ctr ] = plotGMMParameters( Y, est_labels, Mu, Sigma, varargin )
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here

if nargin == 5
    handle = varargin{1};    
else
    handle = figure('Color', [1 1 1]);
end
M = size(Y,1);
h_gmm = [];h_ctr = [];
% Plot Gaussians on Projected Data
if (M == 2) || (M == 3)
    idx_label   = est_labels;   
    pred_clust = size(Mu,2);
    
    if M==2    
        for jj=1:pred_clust
            clust_color = [rand rand rand]  ;
            if nargin < 5
                scatter(Y(1,idx_label==jj),Y(2,idx_label==jj), 50, clust_color, 'filled'); hold on;
            end
           [h_gmm_, h_ctr_ ] = plotGMM(Mu(:,jj), Sigma(:,:,jj), clust_color, 1);
           h_gmm = [h_gmm h_gmm_];
           h_ctr = [h_ctr h_ctr_];           
            alpha(.5)
        end 
        box on
        grid on
        colormap(hot)
        grid on
    end

    if M==3
        subplot(3,1,1)
        clust_color = zeros(length(pred_clust),3);
        for jj=1:pred_clust
            clust_color(jj,:) = [rand rand rand];
            scatter(Y(1,idx_label==jj),Y(2,idx_label==jj), 50, clust_color(jj,:), 'filled');hold on  
            plotGMM(Mu(1:2,jj), Sigma(1:2,1:2,jj), clust_color(jj,:), 1);
            alpha(.5)
        end
        xlabel('y_1');ylabel('y_2');
        axis auto
        colormap(hot)
        grid on
        
        subplot(3,1,2)
        for jj=1:pred_clust
            scatter(Y(1,idx_label==jj),Y(3,idx_label==jj), 50, clust_color(jj,:), 'filled');hold on  
            plotGMM(Mu([1 3],jj), Sigma([1 3],[1 3],jj), clust_color(jj,:), 1);
            alpha(.5)
        end
        xlabel('y_1');ylabel('y_3');
        axis auto
        colormap(hot)
        grid on
        
        subplot(3,1,3)
        for jj=1:pred_clust
            scatter(Y(2,idx_label==jj),Y(3,idx_label==jj), 50, clust_color(jj,:), 'filled');hold on  
            plotGMM(Mu(2:3,jj), Sigma(2:3,2:3,jj), clust_color(jj,:), 1);
            alpha(.5)
        end
        xlabel('y_2');ylabel('y_3');
        axis auto
        colormap(hot)
        grid on
        
    end
end

end

