function [h] = plot_gp_variance_2d(limits, gp_handle, train_inputs)
nx = 400; ny = 400;
axlim = limits;
ax_x=linspace(axlim(1),axlim(2),nx); %computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); %computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y); %meshing the input domain
x=[x_tmp(:), y_tmp(:)]';

[ymu ys2] = feval(gp_handle,train_inputs',ones(size(train_inputs',1),1),x');
z_tmp = reshape(ys2,nx,ny);

h = pcolor(x_tmp,y_tmp,reshape(ys2,nx,ny));
% h = surf(x_tmp,y_tmp,-0.01*ones(nx,ny),reshape(ys2,nx,ny));
set(h,'linestyle','none');
load whiteCopperColorMap;
colormap(flipud(cm));
colorbar;
caxis([min(min(z_tmp)), max(max(z_tmp))]);
end