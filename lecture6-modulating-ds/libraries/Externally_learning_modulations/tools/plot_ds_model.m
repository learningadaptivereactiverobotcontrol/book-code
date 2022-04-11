function [h,xd] = plot_ds_model(fig, ds, limits, varargin)

quality='medium';

if nargin > 4
    quality = varargin{1};
end

if strcmpi(quality,'high')
    nx=400;
    ny=400;
elseif strcmpi(quality,'medium')
    nx=200;
    ny=200;
else
    nx=10;
    ny=10;
end

axlim = limits;
ax_x=linspace(axlim(1),axlim(2),nx); %computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); %computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y); %meshing the input domain
x=[x_tmp(:), y_tmp(:)]';
xd = feval(ds, x);
h = streamslice(x_tmp,y_tmp,reshape(xd(1,:),ny,nx),reshape(xd(2,:),ny,nx),4,'method','cubic');
set(h,'LineWidth', 0.75)
% set(h,'color',[0.0667  0.0667 0.0667]);
end