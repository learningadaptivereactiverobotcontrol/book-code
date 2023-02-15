function [] = simulate_passiveDS_GUI(hfig, robot, base, ds, target, dt, varargin)

% select our figure as gcf
% figure(hfig);
hold on

set(hfig,'WindowButtonDownFcn',@(h,e)button_clicked(h,e));
set(hfig,'WindowButtonUpFcn',[]);
set(hfig,'WindowButtonMotionFcn',[]);
hp = gobjects(0);
          
          
if exist('hd','var'), delete(hd); end
if exist('hx','var'), delete(hx); end

    % Setting robot to starting point
    disp('Select a starting point for the simulation...')
    disp('Once the simulation starts you can perturb the robot with the mouse to get an idea of its compliance.')
    
    infeasible_point = 1;
    while infeasible_point
        try
            xs = get_point(hfig) - base;
            % Another option (Start around demonstrations) :
            % xs  =  Data(1:2,1) - base + 0.15*randn(1,2)'
            qs = simple_robot_ikin(robot, xs);
            robot.animate(qs);
            infeasible_point = 0;
        catch
            warning('could not find a feasible joint space configuration. Please choose another point in the workspace.')
        end
    end
    
    % Run Simulation
    if nargin == 6
        [hd, hx] = simulation_passive_control_GUI(hfig, robot, base, ds, target, qs, dt);
    else
        struct_stiff = varargin{1};
        [hd, hx] = simulation_passive_control_GUI(hfig, robot, base, ds, target, qs, dt, struct_stiff);
    end

    fprintf('Simulation ended.\n')
    

end

