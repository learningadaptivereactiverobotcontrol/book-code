function simulate_lmds(target, limits, fig1, influence, phi_c, A, modulationCenter)

if exist("modulationCenter", "var") == 0
    useUI = true;
else
    useUI = false;
end

ls=influence;

% Plot Attractor
scatter(target(1),target(2),50,[0 0 0],'+'); hold on;

% Construct and plot chosen Linear DS
ds_lin = @(x) lin_ds(x,target, A);
hs = plot_ds_model(fig1, ds_lin, target, limits,'high'); hold on;
% axis tight
title('Original Linear Dynamics $\dot{x}=f_o(x)$', 'Interpreter','LaTex')

% Center of Local Activation
axis(limits);
if useUI
    reshape_btn = uicontrol('Position',[100 20 110 25],'String','Reshape',...
    'Callback','uiresume(gcbf)');

    display('Select Center of Modulation')
    c = get_point(fig1);
else
    c = modulationCenter;
end

% Influence of Local Activation
hold on;
scatter(c(1),c(2),10,[1 0 0],'filled')
exp_funct = @(x) exp_loc_act(ls, c, x);
plot_exp_2d(fig1, exp_funct); hold on;


% Define our reshaped dynamics
reshaped_ds = @(x) lmds_2d(ds_lin, exp_funct, x, phi_c);

if useUI
    display('Press Button to Reshape DS')
    uiwait(gcf);
    delete(reshape_btn)
end

% Delete lin DS model
delete(hs)


% Plot Reshaped dynamics
hs = plot_ds_model(fig1, reshaped_ds, target, limits, 'high');
drawnow;
hold off;

title('Reshaped Dynamics $\dot{x}=f(x)=M(x)f_o(x)$ ', 'Interpreter','LaTex')
display('Reshaping Done.');
end

