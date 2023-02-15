function h = EnergyContour(Vxf,D,varargin)
% Syntax:
%
%       h = EnergyContour(Vxf,D,varargin)
%
% This function computes the energy value at a point x, given the energy
% (Lyapunov) function Vxf. When xd is passed as an empty variable, it also
% provides the energy gradient (i.e. Vdot = dV). Otherwise, it computes the
% rate of change in energy by moving along xd.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         Copyright (c) 2014 Mohammad Khansari, LASA Lab, EPFL,       %%%
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
% S.M. Khansari-Zadeh and A. Billard (2014), "Learning Control Lyapunov Function
% to Ensure Stability of Dynamical System-based Robot Reaching Motions." 
% Robotics and Autonomous Systems, vol. 62, num 6, p. 752-765.
%
% To get latest update of the software please visit
%                          http://cs.stanford.edu/people/khansari/
%
% Please send your feedbacks or questions to:
%                          khansari_at_cs.stanford.edu

quality='low';
b_plot_stream = false;
b_plot_color = true;
b_plot_contour = true;
sp = [];
countour_levels = [];
for i=1:length(varargin)
    if ~isempty(varargin{i})
        switch i
            case 1
                quality = varargin{1};
            case 2
                b_plot_stream = varargin{2};
            case 3
                sp = varargin{3};
            case 4
                countour_levels = varargin{4};
            case 5
                b_plot_color = varargin{5};
        end
    end
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

x = linspace(D(1),D(2),nx);
y = linspace(D(3),D(4),ny);
[X Y] = meshgrid(x,y);
x = [X(:) Y(:)]';

if b_plot_stream
    [V dV] = computeEnergy(x,[],Vxf);
else
    V = computeEnergy(x,[],Vxf);
end

if isempty(countour_levels)
    countour_levels = linspace(0,log(max(V)),40);
    countour_levels = exp(countour_levels);
    if max(V)>40
        countour_levels = round(countour_levels);
    end
end
V = reshape(V,ny,nx);
if isempty(sp)
    figure
    sp = gca;
end

hold on

if b_plot_color
%     pcolor(X,Y,V)
%     shading interp
% %     colormap jet
%     colormap pink
%     ca = caxis;
%     ca(1) = 0;
%     caxis(ca);
    colormap autumn %pink autumn
    mesh(X,Y,V)
    shading interp
end

if b_plot_contour
%     [~,h] = contour(sp,X,Y,log(V),countour_levels);
    [~,h] = contour(sp,X,Y,V,countour_levels);
    
    
%     figure;
%     contourf(X,Y,V);
%     colormap(bone)
%     shading interp; alpha 0.8
    
%     set(h,'ShowText','on','color','k','labelspacing',200);%,'fill','on'
%     set(h,'ShowText','on','TextStep',get(h,'LevelStep')*2,'color','g')
end

if b_plot_stream
    h_S = streamslice(sp,X,Y,reshape(-dV(1,:),ny,nx),reshape(-dV(2,:),ny,nx),0.5,'method','cubic');
    set(h_S,'color','m');
end
plot(sp,0,0,'k*','markersize',12,'linewidth',2)
axis(sp,'equal');axis(sp,'tight');box(sp,'on')


% grid on