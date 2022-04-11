function [h_exp] = plot_exp_2d(fig, exp_handle)

nx = 400; ny = 400;

handle(fig);
axlim = [get(gca,'Xlim'), get(gca, 'Ylim')];
ax_x=linspace(axlim(1),axlim(2),nx); %computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); %computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y);  %meshing the input domain
x=[x_tmp(:), y_tmp(:)]';
[h_x] = feval(exp_handle,x');
h_exp = reshape(h_x,nx,ny);


% Cap the plotting function
% h_exp(h_exp < 0) = 0;

hold on;
% h = pcolor(x_tmp,y_tmp,h_exp);
h = surf(x_tmp,y_tmp,-0.01*ones(nx,ny),h_exp);
set(h,'linestyle','none');
load whiteCopperColorMap;
colormap(cm);
% colorbar;
caxis([min(min(h_exp)), max(max(h_exp))]);

end