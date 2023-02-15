function plotStreamLines(Priors,Mu,Sigma,D,varargin)
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
d = size(Sigma,1);
if d/2~=2
    disp('This function can only be used for 2D models!')
    return
end

quality='low';
if ~isempty(varargin)
	quality=varargin{1};
end

if strcmpi(quality,'high')
    nx=600;
    ny=600;
elseif strcmpi(quality,'medium')
    nx=400;
    ny=400;
else
    nx=200;
    ny=200;
end

ax.XLim = D(1:2);
ax.YLim = D(3:4);
ax_x=linspace(ax.XLim(1),ax.XLim(2),nx); %computing the mesh points along each axis
ax_y=linspace(ax.YLim(1),ax.YLim(2),ny); %computing the mesh points along each axis
[x_tmp y_tmp]=meshgrid(ax_x,ax_y); %meshing the input domain
x=[x_tmp(:) y_tmp(:)]';
z=zeros(1,nx*ny);

xd = GMR(Priors,Mu,Sigma,x,1:d/2,d/2+1:d); %compute outputs
streamslice(x_tmp,y_tmp,reshape(xd(1,:),ny,nx),reshape(xd(2,:),ny,nx),1,'method','cubic')
axis([ax.XLim ax.YLim]);box on