function simulate_lmds(target, limits, influence, phi_c, A, upper, Cof)

fig1 = figure();
ax1 = axes;
hold on;
axis equal;
title('Feasible Robot Workspace','Interpreter','LaTex')
% Plot Attractor
scatter(target(1),target(2),50,[0 0 0],'+'); hold on;

reshape_btn = uicontrol('Position',[100 20 110 25],'String','Reshape',...
    'Callback','uiresume(gcbf)');

% Plot Attractor
scatter(target(1),target(2),50,[0 0 0],'+'); hold on;

% Construct and plot chosen Linear DS
ds_lin = @(x) lin_ds(x,target, A);
hs = plot_ds_model(fig1, ds_lin, limits,'high'); hold on;
% axis tight
title('Original Linear Dynamics $\dot{x}=f_o(x)$', 'Interpreter','LaTex')

disp('Select Center of Modulation');
% Center of Local Activation
axis(limits);
c = get_point(fig1);
% Influence of Local Activation
hold on;
scatter(c(1),c(2),10,[1 0 0],'filled')

exp_funct = @(x) exp_loc_act(influence, c, x);
plot_exp_2d(fig1, exp_funct); hold on;

exter_funct = @(x) soft_step_fun(upper, x, target, Cof);
plot_poly(fig1, exter_funct,limits); hold on


% Define our reshaped dynamics
reshaped_ds = @(x) lmds_2d(ds_lin, exp_funct, exter_funct, x, phi_c);
disp('Press Button to Reshape DS')
uiwait(gcf);

% Delete lin DS model
delete(hs)


% Plot Reshaped dynamics
hs = plot_ds_model(fig1, reshaped_ds, limits, 'high');
title(ax1,'Externally Reshaped Dynamics $\dot{x}=f(x)=M(x,h(s))f_o(x)$ ',...
      'Interpreter','LaTex')
disp('Reshaping Done.');
delete(reshape_btn)
end

