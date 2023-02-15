function [hd, hs, hr, x_sim] = visualizeEstimatedDS(Xi_ref, ds_fun, ds_plot_options)
fig1 = figure('Color',[1 1 1]);
M = size(Xi_ref,1);

% Parse Options
plot_repr   = ds_plot_options.sim_traj;
x0_all      = ds_plot_options.x0_all;

if M == 3
    plot_2D_only = 0;
    
    if isfield(ds_plot_options,'dimensions')
        plot_2D_only = 1;
        dimensions = ds_plot_options.dimensions;
        attractor  = ds_plot_options.attractor;
    else
        init_type    = ds_plot_options.init_type;
        nb_pnts      = ds_plot_options.nb_points;
        plot_volume  = ds_plot_options.plot_vol;
    end
    
end

% Simulate trajectories and plot them on top
if plot_repr
    opt_sim = [];
    opt_sim.dt    = 0.005;   
    opt_sim.i_max = 10000;
    opt_sim.tol   = 0.005;
    opt_sim.plot  = 0;
    [x_sim, ~]    = Simulation(x0_all ,[],ds_fun, opt_sim);
else
    x_sim = [];
    hr = [];
end

% Plot Streamlines
if M == 2 
    [hd] = scatter(Xi_ref(1,:),Xi_ref(2,:),10,[1 0 0],'filled'); hold on
    if isfield(ds_plot_options,'limits')
        limits_ = ds_plot_options.limits;
    else
        limits = axis;
        limits_ = limits + [-0.015 0.015 -0.015 0.015];
    end
    [hs] = plot_ds_model(fig1, ds_fun, [0 0]', limits_,'medium'); hold on;
    axis(limits_)
    box on
    grid on
    xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
    ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
    
    % Plot simulated trajectories
    if plot_repr
        [hr] = scatter(x_sim(1,:),x_sim(2,:),10,[0 0 0],'filled'); hold on
    end
    
elseif M == 3
    
    if plot_2D_only
        Xi_ref_2d = Xi_ref(dimensions,:);
        
        % Plot Demonstrations in red
        [hd] = scatter(Xi_ref_2d(1,:),Xi_ref_2d(2,:),10,[1 0 0],'filled'); hold on
        if isfield(ds_plot_options,'limits')
            limits_ = ds_plot_options.limits;
        else
            limits = axis;
            limits_ = limits + [-0.015 0.015 -0.015 0.015];
        end
        
        % Plot Streamlines of 2D slice in blue
        [hs] = plot_ds_model_3D_2Dslice(fig1, ds_fun, attractor, limits_, dimensions, 'medium'); hold on;       
        axis(limits_)                
        box on
        grid on
        xlabel('$\xi_{d_1}$','Interpreter','LaTex','FontSize',20);
        ylabel('$\xi_{d_2}$','Interpreter','LaTex','FontSize',20);
        
        % Plot simulated trajectories
        if plot_repr
            [hr] = scatter(x_sim(dimensions(1),:),x_sim(dimensions(2),:),10,[0 0 0],'filled'); hold on                        
        end
        
    else
        % Plot Demonstrations in red
        [hd] = plot3(Xi_ref(1,:),Xi_ref(2,:),Xi_ref(3,:),'r.','markersize',10,'DisplayName','Demonstration dataset'); hold on;
        
        % Simulate trajectories and plot them on top
        if plot_repr
            [hr] = plot3(x_sim(1,:),x_sim(2,:),x_sim(3,:),'k.','markersize',10,'DisplayName','Learned DS'); hold on;
        end

        % Compute Start Locations for Streamlines
        start_pnts =  sample_initial_points(x0_all, nb_pnts, init_type, plot_volume);
        limits = axis;
        
        scatter3(start_pnts(1, 1), start_pnts(1, 2), start_pnts(1, 3), 25, 'MarkerEdgeColor','k','MarkerFaceColor',[0 .25 1], 'DisplayName','Learned DS random starting point');

        legend('AutoUpdate', 'off')
        
        
        
        % Plot Streamlines in blue
        [hs] = plot_ds_model_3D(fig1, ds_fun, [0;0;0], limits, start_pnts, 'low'); hold on;
        
        
        axis equal  
        
    end
end

end