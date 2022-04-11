function [h,xd] = plot_ds_model_3D(fig, ds, target, limits, start_locations,varargin)

quality='medium';

if nargin > 5
    quality = varargin{1};
end

if strcmpi(quality,'high')
    nx=400;
    ny=400;
    nz=400;
elseif strcmpi(quality,'medium')
    nx=200;
    ny=200;
    nz=400;
else
    nx=100;
    ny=100;
    nz=100;
end

axlim = limits;
ax_x=linspace(axlim(1),axlim(2),nx); %computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); %computing the mesh points along each axis
ax_z=linspace(axlim(5),axlim(6),nz); %computing the mesh points along each axis

[x_tmp, y_tmp, z_tmp]=meshgrid(ax_x,ax_y, ax_z); %meshing the input domain
x=[x_tmp(:), y_tmp(:), z_tmp(:)]';
xd = feval(ds, x-repmat(target,1,size(x,2)));
startx = start_locations(:,1);
starty = start_locations(:,2);
startz = start_locations(:,3);
scatter3(target(1),target(2),target(3),50,[0 1 0], 'filled'); hold on;
h = streamline(stream3(x_tmp,y_tmp,z_tmp,reshape(xd(1,:),nx,ny,nz),reshape(xd(2,:),nx,ny,nz),reshape(xd(3,:),nx,ny,nz), startx, starty, startz));
grid on;
xlabel('x');ylabel('y');zlabel('z');

set(h,'LineWidth', 0.75)
set(h,'color',[0.0667  0.0667 0.0667]);
end