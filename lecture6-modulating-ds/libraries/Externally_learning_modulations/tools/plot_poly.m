function plot_poly(fig, exp_handle, limits)
% fig=figure();
% h_s=zeros(200*upper,1);
% i=0;
% for t=0:0.01:2*upper
%     h_s(i+1) = feval(exp_handle,t);
%     i=i+1;
% end
% plot(0:0.01:2*upper,h_s);


nx = 400; ny = 400;

handle(fig);
axlim = [get(gca,'Xlim'), get(gca, 'Ylim')];
ax_x=linspace(axlim(1),axlim(2),nx); %computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); %computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y);  %meshing the input domain
x=[x_tmp(:), y_tmp(:)]';
[h_x] = feval(exp_handle,x);
h_exp = reshape(h_x,nx,ny);


% Cap the plotting function
% h_exp(h_exp < 0) = 0;
view(2)
ax2 = axes;
hold on;
axis equal;
axis(limits);
% h = pcolor(x_tmp,y_tmp,h_exp);
h = surf(x_tmp,y_tmp,-0.5*ones(nx,ny),h_exp,'edgecolor','none','FaceAlpha',0.5);
% colorbar;
colormap(ax2,'gray')
%%Hide the top axes
ax2.Visible = 'off';
ax2.XTick = [];
ax2.YTick = [];
end