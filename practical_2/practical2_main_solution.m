%% Section 1: Create world and define obstacles
clc; clear; close all;
folder = fileparts(which('practical2_main_solution.m'));
addpath(genpath(fullfile(folder, 'library_solution')));
addpath(genpath(fullfile(folder, '..', 'practical_1')));
initialPosition = [-0.3; 0.4; 0.7];
myWorld = Environment(initialPosition);
rho = 0.2;

% ------- Obstacle DS to make obstacles move -------
% Set vel_func to whatever you want, just keep the (x,t) input

% Option 1 - Linear DS
A_obs = [-1, 0, 0; ...
          0, -1, 0; ...
          0, 0, -1];  %% Linear DS with stable attractor 
x0_obs = [0.5; 0.5; 0.5];  
b_obs = -A_obs * x0_obs;   % Attractor coordinate are at Ax+b=0

% Option 2 - Constant velocities matrix
% vel_func = @(x,t) [x_vel; y_vel; z_vel]
 
vel_func = @(x,t) [10*cos(50*t); 0; 0 ];
vel_func2 = @(x,t) (A_obs * x + b_obs); 

% ------- Create obstacles and boundaries -------
% Create spherical boundary
%myWorld.addEllipsoid([1, 1, 1], [0, 0, 0], 0.1);

% Create some obstacles
myWorld.addEllipsoid([0.1, 0.1, 0.1], [0.6, -0.5, 0], rho, vel_func);

%myWorld.addEllipsoid([0.1, 0.2, 0.3], [0, 0.2, 0.5], rho);
myWorld.addCylinder(0.3, [0.4, -0.4, 0]);

%myWorld.addEllipsoid([0.2, 0.1, 0.2], [0, -0.2, 0.5], rho);

myWorld.addPlane([0; 1; 0.5], [0; -0.3; 0], rho);
%myWorld.showGammaIsosurfaces(1:0.25:1.5, true);

myWorld.addEllipsoid([0.1, 0.2, 0.3], [0; 0.2; 0.5],  rho, vel_func2);


%% Section 2: Simulate initial conditions

% Initial state: joint velocity and position
q = myWorld.robot.computeInverseKinematics(initialPosition, myWorld.robot.robot.homeConfiguration, ...
                false, diag([1, -1, -1]));
q_dot = zeros(length(q), 1);

% Create robot and controller structure
usingMPC_ds = false;
if usingMPC_ds
    load("ds_control.mat")
    % If this fails, you need to generate a ds_control.mat in the folder 'practical_1'
    % by running the 'MPC_LPVDS.m' function in the solutions folder.
    dsModulated = @(x) modulatedDS(x, attractor, myWorld, ds_control);
else
    attractor = [0.3; 0; 0.3];
    dsModulated = @(x) modulatedDS(x, attractor, myWorld);
end

% Display initial robot pose
ax = show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);

% Plot target point and DS open-loop path
hold on;
plot3(attractor(1), attractor(2), attractor(3), 'go', 'LineWidth', 3, 'MarkerSize', 20);

opt_sim = []; 
opt_sim.plot  = 0;
opt_sim.dt = 3e-3;
opt_sim.i_max = 1500;
echo off;
[x_sim, ~] = Simulation(initialPosition , [], dsModulated, opt_sim);
plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'r', 'LineWidth', 2);

% Plot initial velocity and position
desired_vel = dsModulated(initialPosition);
plot3(initialPosition(1), initialPosition(2), initialPosition(3), '.k', 'MarkerSize', 10)
quiver3(initialPosition(1), initialPosition(2), initialPosition(3), ...
        desired_vel(1), desired_vel(2), desired_vel(3), 0.1, 'b'); 


% Plot DS open-loop path starting at random points
axisLimit = 0.9*myWorld.axisLimit;
for iTraj = 1:10
    randomPosition = rand(3, 1);
    randomPosition(1) = randomPosition(1)*(axisLimit(2)-axisLimit(1)) + axisLimit(1);
    randomPosition(2) = randomPosition(2)*(axisLimit(4)-axisLimit(3)) + axisLimit(3);
    randomPosition(3) = randomPosition(3)*(axisLimit(6)-axisLimit(5)) + axisLimit(5);
    scatter3(randomPosition(1), randomPosition(2), randomPosition(3))
    [~, x_sim, ~] = evalc('Simulation(randomPosition , [], dsModulated, opt_sim)');

    plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'Color', [1,0.6,0.2,0.6]);
end

drawnow;

legend('','','','','','','','','', ...
        'Target','Open-loop trajectory', 'Current position', 'Desired velocity', 'DS trajectories', 'AutoUpdate','off')

%% Section 3: Simulate closed-loop trajectory
view([90, 0])
% Setup controller and disturbance
ctrl = LimitedJointVelocityController(500, myWorld.robot.robot);
disturbance = CartesianForceDisturbance(5e3, 2);

% Simulation setup and parameters
text(axisLimit(2), axisLimit(4), 0.5 * axisLimit(6),{'\bf Perturbations: ', '\rm A/D keys : X axis ', 'Q/E keys : Y axis ', 'W/S keys : Z axis ', ...
                                                    '', '\bfStop simulation : \rm Space bar'}, 'FontSize', 12);

set(myWorld.figure,'WindowKeyPressFcn', {@KeyboardCb, disturbance});

delta_t = 25e-3;
start = tic;
i = 0;
position = myWorld.robot.computeForwardKinematics(q);
global stopFlag;
stopFlag = false;
while vecnorm(position - attractor) > 0.01 && ~stopFlag
    
    % Display disturbance
    if norm(disturbance.values) > 0 && disturbance.iteration < disturbance.nb_active_iterations
        if disturbance.iteration == 0
            if exist('hline', 'var') && exist('hhead', 'var')
                delete(hline);
                delete(hhead);
            end
            [hline, hhead] = arrow3d(position', position' + 0.5 * disturbance.values'/norm(disturbance.values), ...
                15, 'cylinder', [0.2, 0.2], [20, 10], [1, 0 ,0; 1, 0, 0]);
            drawnow;
        end
        disturbance.iteration = disturbance.iteration + 1;
    else
        disturbance.values = zeros(3,1);
        disturbance.iteration = inf;
    end

    % Integrate forward given current state and control
    [t, y] = ode15s(@(t,y) forwardStateDynamic(t, y, myWorld.robot.robot, dsModulated, disturbance.values, ctrl), ...
        [i * delta_t, (i+1) * delta_t], [q; q_dot]);
    
    q = y(end,1:7)';
    q_dot = y(end,8:14)';
    
    show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);    
    plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10)
    drawnow;
    
    position = myWorld.robot.computeForwardKinematics(q);
    myWorld.endEffectorPosition = position;
    
    % Plot desired velocity from DS
    if mod(i, 4)==0
        desired_vel = dsModulated(position);
        quiver3(position(1), position(2), position(3), desired_vel(1), desired_vel(2), desired_vel(3), 0.2, 'Color', [0.2,0,1,0.9]);   
    end
    
    % Move obstacles if desired
    for k =1:length(myWorld.listOfObstacles)
        myWorld.listOfObstacles(k).moveShape(i*delta_t)
    end

    i = i + 1;
end

myWorld.robot.robot.getTransform(q, 'panda_link8');
time_elapsed = toc(start);
disp("Average computation time: " + time_elapsed / i + " [s] | Total time: " + time_elapsed + " [s]")

%% Simulation function
function dydt = forwardStateDynamic(t, y, robot, ds_control, force, ctrl)

    % Current state
    q = y(1:7);
    q_dot = y(8:14);

    % Desired state
    current_position = tform2trvec(robot.getTransform(q, 'panda_link8'))';
    desired_velocity = ds_control(current_position);   

    % Compute corresponding control torque (PD controller)
    full_jac = robot.geometricJacobian(q, 'panda_link8');
    controlAcceleration = ctrl.computeCommand(desired_velocity, q_dot, full_jac);

    % Compute disturbance
    torque_disturbance = full_jac(4:end,:)' * force;
    
    % Compute state derivative from current state and control
    dydt = [q_dot; ...
            controlAcceleration + torque_disturbance];
end


function KeyboardCb(~, event, force_disturbance)
    global stopFlag;
    if strcmp(event.Key, 'space') % space bar
        stopFlag = true;
        disp("Stopped Simulation")
    end
    force_disturbance.handleKeyEvent(event.Key);
    
end

function xdot = modulatedDS(x, attractor, world, nominalDS)

    if nargin < 4
        xdot_nominal = -4*eye(3)*(x-attractor);
    else
        xdot_nominal = nominalDS(x);
    end

    % Initialize modulation matrix and average obstacle velocity
    M_tot = eye(3);
    meanObstacleVelocity = zeros(3, 1);

    % Create the array of gamma values from the list world.listOfObstacles
    % for convenience
    kObst = length(world.listOfObstacles);
    gamma = inf(1, kObst);
    for k = 1:kObst
        gamma(k) = max([world.listOfObstacles(k).gammaDistance(x(1), x(2), x(3)) - world.endEffectorRadius, 1]);
    end
    
    % Go over each obstacle, compute the modulation matrix and apply it to
    % the initial ds velocity xdot_modulated
    for k = 1:kObst
        
        % Compute distance function
        if kObst > 1
            weight = (gamma - 1)./( (gamma - 1) + (gamma(k) - 1) );
            weight = prod(weight(1:k-1))*prod(weight(k+1:end));
        else
            weight = 1;
        end

        % Compute modulation matrix 
        gammaCorrected = abs(gamma(k))^(1/world.listOfObstacles(k).rho);
        D = diag([1 - (weight / gammaCorrected), 1 + (weight / gammaCorrected), 1 + (weight / gammaCorrected)]);
    
        % Compute normal and tangent on ellipse
        normal = world.listOfObstacles(k).gradientGamma(x(1), x(2), x(3));
        normal = normal / vecnorm(normal);
        tangent1 = [0; -normal(3); normal(2)];
        tangent2 = cross(normal, tangent1);
    
        E = [normal, tangent1, tangent2];
        M = E * D / E;
        M_tot = M * M_tot;

        % Weighted average of the obstacle velocities
        meanObstacleVelocity = meanObstacleVelocity + weight * world.listOfObstacles(k).velocity;
    end    

    xdot = M_tot * (xdot_nominal - meanObstacleVelocity) + meanObstacleVelocity;
end

function weight = computeWeight(x, y, z, obstacle, world)
    
    kObst = length(world.listOfObstacles);
    gamma = inf(length(x), kObst);
    for k = 1:kObst
        gamma(:, k) = world.listOfObstacles(k).gammaDistance(x, y, z);
    end
    
    weight = (gamma - 1)./( (gamma - 1) + (gamma(:, obstacle) - 1) );
    weight = prod(weight(:, 1:obstacle-1), 2).*prod(weight(:, obstacle+1:end), 2);

end