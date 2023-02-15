close all
clc
% clear
%% Unstable systems analysis
addpath([pwd, '/Unstable_systems_analysis']);
load('matlab.mat','Input_S')
% Training_N=[1,9,12,13,10,7,17,4,2,5,15]; %'matlab.mat'
% Testing_N=[3,6,8,11,14,16,18,19,20];
% Training_N=[1,2,3,6,11,13,14,16,17,18,20]; % 'matlab_2.mat'
% Testing_N=[4,5,7,8,9,10,12,15,19];
Training_N=[3,4,5,7,8,9,11,12,16,17,20]; % 'matlab_3.mat'
Testing_N=[1,2,6,10,13,14,15,18,19];
dt = 0.01; %The time step of the demonstrations
tol_cutting = 1; % A threshold on velocity that will be used for trimming demos
options.K=3;  %Number of Gaussian funcitons
for i=1:size(Training_N,2)
    Data_Set{1,i}=Input_S{1,Training_N(1,i)}'; %#ok<SAGROW>
end
for i=1:size(Testing_N,2)
    Data_Set_T{1,i}=Input_S{1,Testing_N(1,i)}'; %#ok<SAGROW>
end
close all
% for i=1:20
%    plot(Input_S{1,i}(:,1),Input_S{1,i}(:,2),'DisplayName',sprintf('%f', i),'LineWidth',i)
%    hold on
% end
[~ , ~, DatA, ~] = preprocess_demos(Data_Set,dt,tol_cutting);
[~ , ~, DatA_T, ~] = preprocess_demos(Data_Set_T,dt,tol_cutting);
for counter=1:10
    counter
    options.K=5;  %Number of Gaussian funcitons
    [Unstable_EM{counter}.prior,Unstable_EM{counter}.Mu,Unstable_EM{counter}.Sigma,Unstable_EM{counter}.A,Unstable_EM{counter}.b,Ustable_ini.prior,Ustable_ini.Mu,Ustable_ini.Sigma,time_EM(counter)]=Traing_EM(DatA,options);
    [Unstable_con{counter}.prior,Unstable_con{counter}.Mu,Unstable_con{counter}.Sigma,Unstable_con{counter}.A,time_CON(counter)]=Learn_The_convex_problem(Ustable_ini.prior,Ustable_ini.Mu,Ustable_ini.Sigma,DatA);
    ERROR_EM(counter)=Cross_Validation(DatA,Unstable_EM{counter}.prior,Unstable_EM{counter}.Mu,Unstable_EM{counter}.Sigma,Unstable_EM{counter}.A);
    ERROR_EM_T(counter)=Cross_Validation(DatA_T,Unstable_EM{counter}.prior,Unstable_EM{counter}.Mu,Unstable_EM{counter}.Sigma,Unstable_EM{counter}.A);
    ERROR_CON(counter)=Cross_Validation(DatA,Unstable_con{counter}.prior,Unstable_con{counter}.Mu,Unstable_con{counter}.Sigma,Unstable_con{counter}.A);
    ERROR_CON_T(counter)=Cross_Validation(DatA_T,Unstable_con{counter}.prior,Unstable_con{counter}.Mu,Unstable_con{counter}.Sigma,Unstable_con{counter}.A);
end
