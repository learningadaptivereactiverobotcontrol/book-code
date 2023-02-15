function [init_points] =  sample_initial_points(x0_all, nb_points, type, plot_volume)

% Auxiliary Variable
[M, ~] = size(x0_all);

% Output Variable
init_points = zeros(M, nb_points);

% Estimate point distribution
[ V, D, init_mu ] = my_pca( x0_all );

switch type
    case 'cube'        
        % Do PCA on the points to project the points to 
        % an axis aligned embedding
        [A_y, y0_all] = project_pca(x0_all, init_mu, V, M);
        
        % Find ranges of aligned points
        Ymin_values   = min(y0_all,[],2);
        Ymin_values = Ymin_values + [0; 0.25*Ymin_values(2); 0.25*Ymin_values(3)];
        Yrange_values = range(y0_all,2);               
        Yrange_values = Yrange_values + [0; 0.25*Yrange_values(2); 0.25*Yrange_values(3)];
        
        % Uniformly sample points within the ranges
        init_points_y = Ymin_values(:,ones(1,nb_points)) + rand(nb_points,M)'.*(Yrange_values(:, ones(1,nb_points)));
        
        % Project back to original manifold
        init_points = reconstruct_pca(init_points_y, A_y, init_mu)';
        
        if plot_volume
            [rotmat,cornerpoints,volume,surface] = minboundbox(init_points(:,1),init_points(:,2),init_points(:,3));
            plotminbox(cornerpoints,[0.5 0.5 0.5]); 
            hold on;
        end
        
    case 'ellipsoid'
        D_mod = ((D + 1e-3*eye(M))*diag([3,3,3]));
        init_sigma = V*D_mod*V';
        init_points  = draw_from_ellipsoid(init_sigma, init_mu, nb_points);
        
        if plot_volume
            scale = 3;
            [x,y,z] = created3DgaussianEllipsoid(init_mu,V,D_mod + diag([0,D_mod(2,2)*1.75,D_mod(3,3)*1.75]), scale);
            surf(x, y, z, 'FaceColor',[0.5 0.5 0.5], 'FaceAlpha', 0.25, 'FaceLighting','phong','EdgeColor','none');
            camlight
            hold on;
        end
        
end
end