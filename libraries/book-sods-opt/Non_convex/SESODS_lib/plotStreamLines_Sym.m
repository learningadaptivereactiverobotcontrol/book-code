function plotStreamLines_Sym(Priors_0_P, Mu_0_P, Sigma_0_P ,Priors_O_V ,Mu_0_V ,Sigma_0_V,D,Time,varargin)
% plotStreamLines() : Plots stream lines of the learned model.
% plotStreamLines(quality) : Plots stream lines of the learned model with
%           the speicifed quality. The possible values for quality are:
%           quality = {'low', 'medium', or 'high'} [default='low']
%
% This function can be only used for 2D models.
%
% Optional input variable -------------------------------------------------
%   o quality: An string defining the quality of the plot. It can have one
%              of the following value: 'low', 'medium', or 'high' [default='low']
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2010 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The program is free for non-commercial academic use. Please contact the
% author if you are interested in using the software for commercial purposes.
% The software must not be modified or distributed without prior permission
% of the authors. Please acknowledge the authors in any academic publications
% that have made use of this code or part of it. Please use this BibTex
% reference:
% 
% S. M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear Dynamical 
% Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011.
%
% To get latest upadate of the software please visit
%                          http://lasa.epfl.ch/khansari
%
% Please send your feedbacks or questions to:
%                           mohammad.khansari_at_epfl.ch

%%
d = size(Sigma_0_P,1);
if d/2~=2
    disp('This function can only be used for 2D models!')
    return
end

quality='low';
if ~isempty(varargin)
	quality=varargin{1};
end

if strcmpi(quality,'high')
    nx=16;
    ny=16;
elseif strcmpi(quality,'medium')
    nx=8;
    ny=8;
else
    nx=4;
    ny=4;
end

ax.XLim = D(1:2);
ax.YLim = D(3:4);
ax_x=linspace(ax.XLim(1),ax.XLim(2),nx); %computing the mesh points along each axis
ax_y=linspace(ax.YLim(1),ax.YLim(2),ny); %computing the mesh points along each axis
[x_tmp y_tmp]=meshgrid(ax_x,ax_y); %meshing the input domain
x=[x_tmp(:) y_tmp(:)]';
z=zeros(1,nx*ny);
dt=0.01;
Size=size(x,2);
options = odeset('RelTol',1e-6,'AbsTol',1e-6);
for j=1:Size
    Handle_X=x(:,j);
    Handle_DX=[0;0];
    Xrobot_object_initial=transpose([Handle_X ;Handle_DX]);
    [T1,x_hybrid] = ode45(@(t,xi_r) SE_Second(t,xi_r,Priors_0_P,Mu_0_P,Sigma_0_P,Priors_O_V, Mu_0_V, Sigma_0_V),[0:0.01:1.1*Time],Xrobot_object_initial,options);
    X(:,:,j)=x_hybrid(:,1:d/2);
    DX(:,:,j)=x_hybrid(:,d/2+1:d);
%     plot(X(:,1,j),X(:,2,j))
%     hold on
end
figure1 = figure;
% Create subplot
subplot1 = subplot(2,1,1,'Parent',figure1);
box(subplot1,'on');
hold(subplot1,'on');

% Create ylabel
ylabel('$\dot{\xi}(1,2)$','FontSize',20,'Interpreter','latex');

% Create xlabel
xlabel('$\dot{\xi}(1,1)$','FontSize',20,'Interpreter','latex');

% Create title
title('Velocity');

% Create subplot
subplot2 = subplot(2,1,2,'Parent',figure1);
box(subplot2,'on');
hold(subplot2,'on');

% Create ylabel
ylabel('$\xi(1,2)$','FontSize',20,'Interpreter','latex');

% Create xlabel
xlabel('\xi(1,1)','FontSize',20);

% Create title
title('Position');
for i=1:size(X,1)
    if rem(i,2)==0
        for j=1:16 
            A=subplot(2,1,1);
            plot(DX(i,1,j),DX(i,2,j),'.','Color',[(j/16)^2 0 1-j/16]);
            hold on
            A=subplot(2,1,2);
            plot(X(i,1,j),X(i,2,j),'.','Color',[(j/16)^2 0 1-j/16]);
            hold on
        end
    end
    pause(0.01)
end

function dy = SE_Second(t,xi_r,Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V)
K=size(Sigma_0_P);
if max(size(K))==2
    K(1,3)=1;
end
d=size(xi_r);
d(1,1)=d(1,1)/2;
Input=zeros(d(1,1),1);
DInput=zeros(d(1,1),1);
Numerator=zeros(1,K(1,3));
A=zeros(d(1,1),d(1,1),K(1,3));
%%% Position
for i=1:d(1,1)
    Input(i,1)=xi_r(i,1);
end
for i=1:d(1,1)
    DInput(i,1)=xi_r(i+d(1,1),1);
end

Output=0;
Denominator=0; 
for j=1:K(1,3)
    Sigmainput=Sigma_0_P(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_P(1:d(1,1),j);
    PI=Priors_0_P(1,j);
    Numerator(1,j)=(1/sqrt(((2*pi)^(K(1,3)+d(1,1)))*abs(det(Sigmainput))))*exp(-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput)));
    Denominator=Denominator+PI*Numerator(1,j);
end
if Denominator==0
   Denominator=realmin;
end

for o=1:K(1,3) 
    Sigmainput=Sigma_0_P(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output=Sigma_0_P(d(1,1)+1:2*d(1,1),1:d(1,1),o);
    PI=Priors_0_P(1,o);
    A(:,:,o)=SigmaInput_Output/(Sigmainput);
    Output=Output+(PI*Numerator(1,o)/Denominator)*(A(:,:,o)*(Input));
end
%% Velocity
Input=DInput;
DOutput=0;
Denominator=0;
for j=1:K(1,3)
    Sigmainput=Sigma_0_V(1:d(1,1),1:d(1,1),j);
    Muinput=Mu_0_V(1:d(1,1),j);
    PI=Priors_0_V(1,j);
    Numerator(1,j)=(1/sqrt(((2*pi)^(K(1,3)+d(1,1)))*abs(det(Sigmainput))))*exp(-0.5*abs((transpose(Input-Muinput)/(Sigmainput))*(Input-Muinput)));
    Denominator=Denominator+PI*Numerator(1,j);
end
if Denominator==0
   Denominator=realmin;
end

for o=1:K(1,3)
    Sigmainput=Sigma_0_V(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output=Sigma_0_V(d(1,1)+1:2*d(1,1),1:d(1,1),o);
    PI=Priors_0_V(1,o);
    A(:,:,o)=SigmaInput_Output/(Sigmainput);
    DOutput=DOutput+(PI*Numerator(1,o)/Denominator)*(A(:,:,o)*(Input));
end
Dx_main=Output+DOutput;
dy=([DInput;Dx_main]);