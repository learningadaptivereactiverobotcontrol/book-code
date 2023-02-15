% function [prior_out,Mu_out,Sigma_out,prior_N,Mu_N,Sigma_N,A,Time]=Solve_The_convex_SMEM(Data)
close all
clc
clear
addpath([pwd, '/Unstable_systems_analysis']);
load('matlab.mat','Input_S')
Training_N=[1,9,12,13,10,7,17,4,2,5,15]; %'matlab.mat'
Testing_N=[3,6,8,11,14,16,18,19,20];
% Training_N=[1,2,3,6,11,13,14,16,17,18,20]; % 'matlab_2.mat'
% Testing_N=[4,5,7,8,9,10,12,15,19];
% Training_N=[3,4,5,7,8,9,11,12,16,17,20]; % 'matlab_3.mat'
% Testing_N=[1,2,6,10,13,14,15,18,19];
dt = 1; %The time step of the demonstrations
tol_cutting = 0.0; % A threshold on velocity that will be used for trimming demos
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
[~ , ~, Data, ~] = preprocess_demos(Data_Set,dt,tol_cutting);













Time=0;
counter=1;
d=size(Data,1)/2;
epsilon=10^(-5);
temp=1+2*d+(3*d*d+d)/2;
K0=ceil(size(Data,2)/temp);
% K0=5;
[Priors_0, Mu_0, Sigma_0] = EM_init_kmeans(Data, K0);
options=sdpsettings('solver','gurobi','gurobi.IterationLimit',1000);
H=H_x(Priors_0, Mu_0, Sigma_0,Data(1:d,:));
A = sdpvar(d,d,K0,'full');
b= sdpvar(d,K0,'full');
Fun=0;
for i=1:K0
    Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
end
diff=Fun-Data(d+1:2*d,:);
aux = sdpvar(2,length(diff));
sol=optimize([aux == diff],sum((sum(aux.^2))));
Time=Time+sol.solvertime;
tol(counter)=value(sum((sum(aux.^2))))*K0;
% tol(counter)=value(sum((sum(aux.^2))));
A_V=value(A);
b_V=value(b);
Sigma_handle=Sigma_0;
prior_handle=Priors_0;
Mu_handle=Mu_0;
for i=1:K0
    Sigma_input_output=A_V(:,:,i)*Sigma_handle(1:d,1:d,i);
    Sigma_handle(d+1:2*d,1:d,i)=Sigma_input_output;
    Sigma_handle(1:d,d+1:2*d,i)=Sigma_input_output';
    Mu_handle(d+1:2*d,i)=A_V(:,:,i)*Mu_handle(1:d,i)+b_V(:,i);
end
for i=1:K0
    M_c(i,:)=sum(((A_V(:,:,i)*Data(1:d,:)+repmat(b_V(:,i),1,size(Data,2)))-Data(d+1:2*d,:)).^2,1);  % check this one
end
% for i=1:K0
%     M_c(i,:)=gaussPDF(Data(1:d,:), Mu_handle(1:d,i), Sigma_handle(1:d,1:d,i))';  % check this one
% end
for i=1:K0-1
    for j=i+1:K0
                J_merg(i,j)=M_c(i,:)*M_c(j,:)';
%         J_merg(i,j)=norm(Mu_handle(1:d,i)-Mu_handle(1:d,j));
    end
end
% J_merg(J_merg==0)=2*min(min(J_merg));
% [~, minloc] = max(J_merg(:));
% [minloc_row, minloc_col] = ind2sub(size(J_merg), minloc);
J_merg(J_merg==0)=2*max(max(J_merg));
[~, minloc] = min(J_merg(:));
[minloc_row, minloc_col] = ind2sub(size(J_merg), minloc);
clearvars J_merg M_c Mu_0 Priors_0 Sigma_0
error_P=100;
Discription='Distance'
while (K0>3)
    counter=counter+1;
    K=size(prior_handle,2);
    counter_K=1;
    for i=1:K
        if ((i~=minloc_row)&&(i~=minloc_col))
            Priors_0(1,counter_K)=prior_handle(1,i);
            Mu_0(:,counter_K)=Mu_handle(:,i);
            Sigma_0(:,:,counter_K)=Sigma_handle(:,:,i);
            counter_K=counter_K+1;
        end
    end
    Priors_0(1,counter_K)=prior_handle(1,minloc_row)+prior_handle(1,minloc_col);
    Mu_0(:,counter_K)=(prior_handle(1,minloc_row)*Mu_handle(:,minloc_row)+prior_handle(1,minloc_col)*Mu_handle(:,minloc_col))./(prior_handle(1,minloc_row)+prior_handle(1,minloc_col));
    Sigma_0(:,:,counter_K)=(prior_handle(1,minloc_row)*Sigma_handle(:,:,minloc_row)+prior_handle(1,minloc_col)*Sigma_handle(:,:,minloc_col))./(prior_handle(1,minloc_row)+prior_handle(1,minloc_col));
    K0=K-1;
    H=H_x(Priors_0, Mu_0, Sigma_0,Data(1:d,:));
    A = sdpvar(d,d,K0,'full');
    b= sdpvar(d,K0,'full');
    Fun=0;
    for i=1:K0
        Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
    end
    diff=Fun-Data(d+1:2*d,:);
    aux = sdpvar(2,length(diff));
 fun=sum((sum(aux.^2)));
    sol=optimize([aux == diff],fun);
    Time=Time+sol.solvertime;
        tol(counter)=value(fun)*K0;
%     tol(counter)=value(sum((sum(aux.^2))));
    A_V=value(A);
    b_V=value(b);
    Sigma_handle=Sigma_0;
    prior_handle=Priors_0;
    Mu_handle=Mu_0;
    for i=1:K0
        Sigma_input_output=A_V(:,:,i)*Sigma_handle(1:d,1:d,i);
        %         Sigma_handle(d+1:2*d,1:d,i)=Sigma_input_output;
        Sigma_handle(1:d,d+1:2*d,i)=Sigma_input_output';
        Mu_handle(d+1:2*d,i)=A_V(:,:,i)*Mu_handle(1:d,i)+b_V(:,i);
    end
    for i=1:K0
        M_c(i,:)=sum(((A_V(:,:,i)*Data(1:d,:)+repmat(b_V(:,i),1,size(Data,2)))-Data(d+1:2*d,:)).^2,1);  % check this one
    end
    %     for i=1:K0
    %         M_c(i,:)=gaussPDF(Data(1:d,:), Mu_handle(1:d,i), Sigma_handle(1:d,1:d,i))';  % check this one
    %     end
    for i=1:K0-1
        for j=i+1:K0
                    J_merg(i,j)=M_c(i,:)*M_c(j,:)';
%             J_merg(i,j)=norm(Mu_handle(1:d,i)-Mu_handle(1:d,j));
        end
    end
    %     J_merg(J_merg==0)=2*min(min(J_merg));
    %     [~, minloc] = max(J_merg(:));
    %     [minloc_row, minloc_col] = ind2sub(size(J_merg), minloc);
    J_merg(J_merg==0)=2*max(max(J_merg));
    [~, minloc] = min(J_merg(:));
    [minloc_row, minloc_col] = ind2sub(size(J_merg), minloc);
    Sigma_N{counter}=Sigma_handle;
    Mu_N{counter}=Mu_handle;
    prior_N{counter}=prior_handle;
    clearvars J_merg M_c Mu_0 Priors_0 Sigma_0
    error_P=(tol(counter-1)-tol(counter));
end
Sigma_out=Sigma_handle;
Mu_out=Mu_handle;
prior_out=prior_handle;
keyboard









