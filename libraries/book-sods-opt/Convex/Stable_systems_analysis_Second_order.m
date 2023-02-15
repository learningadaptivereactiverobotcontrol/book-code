close all
clc
clear
addpath([pwd, '/Unstable_systems_analysis']);
addpath([pwd, '/Stable_systems_analysis']);
cd ..
% addpath(genpath([pwd, '/Non_convex']));
% addpath(genpath([pwd, '/YALMIP']));
% addpath(genpath([pwd, '/PENLABv104']));

cd Convex
load('Data/matlab_2.mat','Input_S') % Other options are matlab_3.mat matlab_2.mat
Training_N=[1,2,3]; %'matlab.mat'
Testing_N=[];
for i=1:size(Training_N,2)
    Data_Set{1,i}=Input_S{1,Training_N(1,i)}'; %#ok<SAGROW>
end

%% Initialization
options.dt = 0.5; %The time step of the demonstrations
options.tol_cutting = 0.0000001; % A threshold on velocity that will be used for trimming demos

% Training parameters
options.K = 3; %Number of Gaussian
options.d = 2; % Dimention  of Demostrations
options.I=1; % It's a reduced factor. Leave it one if you want the dynamical system is trainded according to all the data.
options.tol_mat_bias = 10^-18; % A very small positive scalar to avoid  instabilities in Gaussian kernel
options.display = 1;          % An option to control whether the algorithm displays the output of each iterations [default: true]
options.tol_stopping=10^(-6);  % A small positive scalar defining the stoppping tolerance for the optimization solver [default: 10^-10]
options.max_iter = 1000; % Maximum number of iteration for the solver [default: i_max=1000]
options.TolCon = 1e-2;   % Tolerance on the constraint violation, a positive scalar. The default is 1e-6.
options.Normilizing='False'; % If the number of the training data points need to be normalized along the path.
options.Velocity='False'; % Velocity and Position of the demostrations are provided by the user.
% If you put it true, make sure that the name of demostrations are demo_P, and demo_V with same lengh.
% SYM to constract S-SD
options.smoothing='False'; % Recunstracting the demostrations from velocity profile.
options.Method=2; % Two types for initilizing are provied. First one is based on a one mapping from velocities and Positions to accelerartions.
% Second one is based on a seperate mapping from velocities and Positions to accelerartions
options.Method_For_APA='ASYM'; % SYM to use the result of S-DS to coustract a J-SD
%% SESODS

if (strcmp(options.Velocity,'False')==1)
    demos_P=[];
    demos_V=[];
end
% Initialization

[~,~,~,~, ~, ~, Data_P_A, Data_V_A,~,demos_P,demos_V,demos,~]=Initialization(Data_Set,demos_P,demos_V,options);
DatA=[Data_P_A(1:options.d,:);Data_V_A(1:options.d,:);Data_V_A(1:options.d,:);Data_V_A(options.d+1:2*options.d,:)];
[Unstable_EM.prior,Unstable_EM.Mu,Unstable_EM.Sigma,Unstable_EM.A,Unstable_EM.b,~,~,~,~]=Traing_EM(DatA,options);
[Stable.prior,Stable.Mu,Stable.Sigma,Stable.A,time_CON]=Learn_The_convex_Stable_problem_second(Unstable_EM.prior,Unstable_EM.Mu,Unstable_EM.Sigma,DatA);
%% Simulation
figure1 = figure;
axes1 = axes('Parent',figure1);
hold(axes1,'on');
xlabel('X [m]');
ylabel('Y [m]');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'FontSize',24);
for i=1:size(Training_N,2)
   plot(Input_S{1,Training_N(1,i)}(:,1),Input_S{1,Training_N(1,i)}(:,2),'DisplayName','Training set','LineWidth',3,'Color',[1 0 0])
   hold on
end
for i=1:size(Testing_N,2)
   plot(Input_S{1,Testing_N(1,i)}(:,1),Input_S{1,Testing_N(1,i)}(:,2),'DisplayName','Testing set','LineWidth',3,'Color',[0 1 0])
   hold on
end
Simulation(demos,Unstable_EM,Stable)