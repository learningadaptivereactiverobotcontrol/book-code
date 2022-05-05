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
%%%  Example: 2-DOF Robotic arm- impedance controller- Approach one.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example:  2-DOF Robotic arm- impedance controller
%   input -----------------------------------------------------------------
%
%       o robot                : (1 x 1), Robot's specification.                (Mandatory)
%       o Impedance_Inertia    : (2 x 1)(2 x 2), Desired inertia matrices.      (Mandatory)
%       o Impedance_Damping    : (2 x 1)(2 x 2), Desired damping matrices.      (Mandatory)
%       o Impedance_Stifness  : (2 x 1)(2 x 2), Desired stifness matrices.     (Mandatory)
%       o External_force       : (1 x 1), Amplitude of external force.          (Mandatory)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Impedance_controller_approch_one(robot,Impedance_Inertia,Impedance_Damping,Impedance_Stifness,Impedance_mu,External_force)
close all
clc

Time=20; % Duration of the simulation
dt=0.001; % Sample time

%%%%%%%%%%%%%% Impedance paramters, change them!
desired.Inertia1=Impedance_Inertia{1,1};
desired.Damping1=Impedance_Damping{1,1};
desired.Stifness1=Impedance_Stifness{1,1};

desired.Inertia2=Impedance_Inertia{1,2};
desired.Damping2=Impedance_Damping{1,2};
desired.Stifness2=Impedance_Stifness{1,2};

desired.mu1=Impedance_mu{1,1};
desired.mu2=Impedance_mu{1,2};



robot=update_the_robot(robot); %% Calculate the next state of the robot, given the current torque/state

DDq=zeros(2,Time/dt); Dq=zeros(2,Time/dt); q=zeros(2,Time/dt);
DDq_d=zeros(2,Time/dt); Dq_d=zeros(2,Time/dt); q_d=zeros(2,Time/dt);
Tau=zeros(2,Time/dt); time=zeros(1,Time/dt); F_c=zeros(2,Time/dt);
Damping=zeros(1,Time/dt); Stifness=zeros(1,Time/dt);

% Set inital configuration values 
q(:,1) = [robot.q1;robot.q2];
Dq(:,1) = [robot.Dq1;robot.Dq2];
DDq(:,1) = [robot.DDq1;robot.DDq2];

counter=1;
while (counter*dt<Time)
    robot=update_the_robot(robot);
    desired=Update_the_stiffness_damping(robot,desired);
    desired.q=[sin(time(counter));cos(time(counter))];
    desired.Dq=[cos(time(counter));-sin(time(counter))];
    desired.DDq=[-sin(time(counter));-cos(time(counter))];
    DDq_d(:,counter)=desired.DDq;
    Dq_d(:,counter)=desired.Dq;
    q_d(:,counter)=desired.q;
    Damping(counter)=norm(desired.Damping);
    Stifness(counter)=norm(desired.Stifness);
    %%%%%%%%%%%%%% External force, change them!
    if (abs(counter*dt-5.0)<(dt/100))||(abs(counter*dt-14.0)<(dt/100))
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
figure1 = figure;
screensize = get(groot, 'Screensize'); 
figure1.Position = [0.005  * screensize(3), 0.15  * screensize(4), 0.33 * screensize(3), 0.8 * screensize(4)];  
axes1=subplot(3,1,1,'Parent',figure1);
title('Robot parameters, Approach 1','FontSize',12)
hold(axes1,'on');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
plot(time(1,1:end-1),q(1,1:end-1),'LineWidth',1.5,'DisplayName','$ q(1)$','color','#f5f173')
hold on
plot(time(1,1:end-1),q_d(1,1:end-1),'LineWidth',1.5,'DisplayName','$ q_d(1)$', 'LineStyle','--','color','#ed3915')
plot(time(1,1:end-1),q(2,1:end-1),'LineWidth',1.5,'DisplayName','$ q(2)$','color','#c592e0')
hold on
plot(time(1,1:end-1),q_d(2,1:end-1),'LineWidth',1.5,'DisplayName','$ q_d(2)$', 'LineStyle','--','color','#3209e6')
% xlabel('Time [s]','Interpreter','latex');
ylabel('Joint Position [rad]','Interpreter','latex','FontSize',12);
set(gca,'XTickLabel',[]);
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
%% Ploting torque
axes1 = subplot(3,1,2,'Parent',figure1);
hold(axes1,'on');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
plot(time(1,1:end-1),Tau(1,1:end-1),'LineWidth',1.5,'DisplayName','$ \tau(1)$')
hold on
plot(time(1,1:end-1),Tau(2,1:end-1),'LineWidth',1.5,'DisplayName','$ \tau(2)$')
ylabel('Control torque [Nm]','Interpreter','latex','FontSize',12);
set(gca,'XTickLabel',[]);
% ylim(axes1,[-10 10]);
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
%% Ploting external force
axes1 = subplot(3,1,3,'Parent',figure1);
hold(axes1,'on');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
plot(time(1,1:end-1),F_c(1,1:end-1),'LineWidth',1.5,'DisplayName','$J^TF_c(1)$')
hold on
plot(time(1,1:end-1),F_c(2,1:end-1),'LineWidth',1.5,'DisplayName','$J^TF_c(2)$')
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('External torque [Nm]','Interpreter','latex','FontSize',12);
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
%% Ploting Damping
figure2 = figure;
axes1=subplot(2,1,1,'Parent',figure2);
screensize = get(groot, 'Screensize'); 
figure2.Position = [0.005  * screensize(3), 0.02  * screensize(4), 0.33 * screensize(3), 0.35 * screensize(4)];    
title('Impedance variables, Approach 1', 'FontSize', 12);
hold(axes1,'on');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
plot(time(1,1:end-1),Damping(1,1:end-1),'LineWidth',1.5,'DisplayName','$ D(q)$')
% xlabel('Time [s]','Interpreter','latex');
ylabel('Damping [$\frac{N.s}{m}$]','Interpreter','latex','FontSize',10);
set(gca,'XTickLabel',[]);
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
%% Ploting Stifness
axes1 = subplot(2,1,2,'Parent',figure2);
hold(axes1,'on');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
plot(time(1,1:end-1),Stifness(1,1:end-1),'LineWidth',1.5,'DisplayName','$ K(q)$')
xlabel('Time [s]','Interpreter','latex','FontSize',8);
ylabel('Stifness [$\frac{N}{m}$]','Interpreter','latex','FontSize',10);
set(gca,'XTickLabel',[]);
box(axes1,'on');
grid(axes1,'on');
set(axes1,'TickLabelInterpreter','latex');
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex');
figure(figure2)
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


function desired=Update_the_stiffness_damping(robot,desired)
% Update the stifness and the damping matrices
Theta1=(robot.q-desired.mu1)'*(robot.q-desired.mu1);
Theta2=(robot.q-desired.mu2)'*(robot.q-desired.mu2);
sum=Theta1+Theta2;
Theta1=Theta1/sum;
Theta2=Theta2/sum;
desired.Inertia=desired.Inertia1;
desired.Damping=Theta1*desired.Damping1+Theta2*desired.Damping2;
desired.Stifness=Theta1*desired.Stifness1+Theta2*desired.Stifness2;
end