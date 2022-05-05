%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
%   Author: Sina Mirrazavi
%   email:   sina.mirrazavi@epfl.ch
%   website: lasa.epfl.ch
%
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details
%%%  Name of the chapter: Dynamical system based compliant control
%%%  Example: 2-DOF Robotic arm- impedance controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example:  2-DOF Robotic arm- impedance controller
%   input -----------------------------------------------------------------
%
%       o robot                : (1 x 1), Robot's specification.       (Mandatory)
%       o Impedance_Inertia    : (2 x 2), Desired inertia matrix.      (Mandatory)
%       o Impedance_Damping    : (2 x 2), Desired damping matrix.      (Mandatory)
%       o Impedance_Stinfness  : (2 x 2), Desired stifness matrix.     (Mandatory)
%       o External_force       : (1 x 1), Amplitude of external force. (Mandatory)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Impedance_controller_full(robot,Impedance_Inertia,Impedance_Damping,Impedance_Stinfness,External_force)
%close all
clc

Time=20; % Duration of the simulation
dt=0.001; % Integration step


%%%%%%%%%%%%%% Impedance paramters
desired.Inertia=Impedance_Inertia;
desired.Damping=Impedance_Damping;
desired.Stifness=Impedance_Stinfness;

robot=update_the_robot(robot); %% Calculate the next state of the robot, given the current torque/state

DDq=zeros(2,Time/dt); Dq=zeros(2,Time/dt); q=zeros(2,Time/dt);
DDq_d=zeros(2,Time/dt); Dq_d=zeros(2,Time/dt); q_d=zeros(2,Time/dt);
Tau=zeros(2,Time/dt); time=zeros(1,Time/dt); F_c=zeros(2,Time/dt);

% Set inital configuration values 
q(:,1) = [robot.q1;robot.q2];
Dq(:,1) = [robot.Dq1;robot.Dq2];
DDq(:,1) = [robot.DDq1;robot.DDq2];

counter=1;
while (counter*dt<Time)
    robot=update_the_robot(robot);
    desired.q=[sin(time(counter));cos(time(counter))];
    desired.Dq=[cos(time(counter));-sin(time(counter))];
    desired.DDq=[-sin(time(counter));-cos(time(counter))];
    DDq_d(:,counter)=desired.DDq;
    Dq_d(:,counter)=desired.Dq;
    q_d(:,counter)=desired.q;
    %%%%%%%%%%%%%% External force, change them!
    if abs(rem(counter*dt,7.5))<(dt/100)
        for i=1:100
            F_c(:,counter+i)=External_force;
        end
    end
    Tau(:,counter)=update_control_law(robot,desired,F_c(:,counter));
    DDq(:,counter+1)=(robot.Inertia)\(Tau(:,counter)+F_c(:,counter)-robot.Collios*Dq(:,counter)-robot.Gravity);
    Dq(:,counter+1)=Dq(:,counter)+DDq(:,counter+1)*dt;
    q(:,counter+1)=q(:,counter)+Dq(:,counter+1)*dt;
    robot.DDq=DDq(:,counter+1); robot.q=q(:,counter+1); robot.Dq=Dq(:,counter+1);
    robot.tau=Tau(:,counter+1);
    time(counter+1)=time(counter)+dt;
    counter=counter+1;
end


%% Ploting position 
figureFull = figure;
screensize = get(groot, 'Screensize'); 
figureFull.Position = [0.665  * screensize(3), 0.1  * screensize(4), 0.6 * screensize(3), 0.8 * screensize(4)];  
axesFull=subplot(3,1,1,'Parent',figureFull);
title('Full impedance controller' ,'FontSize',12)
hold(axesFull,'on');
box(axesFull,'on');
grid(axesFull,'on');
set(axesFull,'TickLabelInterpreter','latex');
legendFull = legend(axesFull,'show');
set(legendFull,'Interpreter','latex');
plot(time(1,1:end-1),q(1,1:end-1),'LineWidth',1.5,'DisplayName','$ q(1)$', 'Color','#f5f173')
hold on
plot(time(1,1:end-1),q_d(1,1:end-1),'LineWidth',1.5,'DisplayName','$ q_d(1)$', 'LineStyle','--', 'Color','#ed3915')
plot(time(1,1:end-1),q(2,1:end-1),'LineWidth',1.5,'DisplayName','$ q(2)$', 'Color','#c592e0')
hold on
plot(time(1,1:end-1),q_d(2,1:end-1),'LineWidth',1.5,'DisplayName','$ q_d(2)$', 'LineStyle','--', 'Color','#3209e6')
% xlabel('Time [s]','Interpreter','latex');
ylabel('Joint Position [rad]','Interpreter','latex','FontSize',12);
set(gca,'XTickLabel',[]);
box(axesFull,'on');
grid(axesFull,'on');
set(axesFull,'TickLabelInterpreter','latex');
legendFull = legend(axesFull,'show');
set(legendFull,'Interpreter','latex');
%% Ploting torque
axesFull = subplot(3,1,2,'Parent',figureFull);
hold(axesFull,'on');
box(axesFull,'on');
grid(axesFull,'on');
set(axesFull,'TickLabelInterpreter','latex');
legendFull = legend(axesFull,'show');
set(legendFull,'Interpreter','latex');
plot(time(1,1:end-1),Tau(1,1:end-1),'LineWidth',1.5,'DisplayName','$ \tau(1)$')
hold on
plot(time(1,1:end-1),Tau(2,1:end-1),'LineWidth',1.5,'DisplayName','$ \tau(2)$')
ylabel('Control torque [Nm]','Interpreter','latex','FontSize',12);
set(gca,'XTickLabel',[]);
% ylim(axesFull,[-10 10]);
box(axesFull,'on');
grid(axesFull,'on');
set(axesFull,'TickLabelInterpreter','latex');
legendFull = legend(axesFull,'show');
set(legendFull,'Interpreter','latex');
%% Ploting external force
axesFull = subplot(3,1,3,'Parent',figureFull);
hold(axesFull,'on');
box(axesFull,'on');
grid(axesFull,'on');
set(axesFull,'TickLabelInterpreter','latex');
legendFull = legend(axesFull,'show');
set(legendFull,'Interpreter','latex');
plot(time(1,1:end-1),F_c(1,1:end-1),'LineWidth',1.5,'DisplayName','$J^TF_c(1)$')
hold on
plot(time(1,1:end-1),F_c(2,1:end-1),'LineWidth',1.5,'DisplayName','$J^TF_c(2)$')
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('External torque [Nm]','Interpreter','latex','FontSize',12);
box(axesFull,'on');
grid(axesFull,'on');
set(axesFull,'TickLabelInterpreter','latex');
legendFull = legend(axesFull,'show');
set(legendFull,'Interpreter','latex');
end




function Robot=update_the_robot(robot)
% Update the Mass, Collios and the gravity matices.
g=-9.8; % Gravity, Change it!
Robot=robot;
H11=robot.m1*(robot.l1/2)^2+robot.m2*((robot.l1)^2+(robot.l2/2)^2+2*robot.l1*(robot.l2/2)*cos(robot.q2));
H22=robot.m2*(robot.l2/2)^2;
H12=robot.m2*robot.l1*(robot.l2/2)*cos(robot.q2)+robot.m2*(robot.l2/2)^2;
Robot.Inertia=[H11 H12;H12 H22]; % The mass matrix
h=robot.m2*robot.l1*(robot.l2/2)*sin(robot.q2); 
Robot.Collios=[-h*robot.Dq2 -h*robot.Dq1-h*robot.Dq2;robot.Dq1 0];  % The collios matrix
Robot.Gravity=g*[robot.m1*(robot.l1/2)*cos(robot.q1)+robot.m2*((robot.l2/2)*cos(robot.q1+robot.q2)+robot.l1*cos(robot.q1));robot.m2*(robot.l2/2)*cos(robot.q1+robot.q2)];  % The garvity matrix
end

function Tau=update_control_law(robot,desired,F_c)
% Calculate the desired torque command
Tau=robot.Collios*robot.Dq+robot.Gravity+robot.Inertia*desired.DDq...
    -robot.Inertia*inv(desired.Inertia)*(desired.Damping*(robot.Dq-desired.Dq)...
    +desired.Stifness*(robot.q-desired.q))+(robot.Inertia*inv(desired.Inertia)-eye(2))*F_c;

end
