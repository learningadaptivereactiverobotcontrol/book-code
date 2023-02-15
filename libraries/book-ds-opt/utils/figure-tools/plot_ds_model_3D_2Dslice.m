function [h,xd] = plot_ds_model_3D_2Dslice(fig, ds, attractor, limits, dimensions, varargin)

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
    nx=50;
    ny=50;
end

axlim           = limits;
ax_x            = linspace(axlim(1),axlim(2),nx); % computing the mesh points along each axis
ax_y            = linspace(axlim(3),axlim(4),ny); % computing the mesh points along each axis
[d1_tmp, d2_tmp]= meshgrid(ax_x,ax_y);  % meshing the input domain

d3_tmp = zeros(size(d1_tmp));
d1_vec = d1_tmp(:);
d2_vec = d2_tmp(:);

dimensions

x = repmat(attractor,[1 length(d1_vec)]);

if dimensions(1) == 1
    x(1,:) = d1_vec';
    fprintf('First dimension is x \n');
elseif dimensions(1) == 2
    x(2,:) = d1_vec';
    fprintf('First dimension is y \n');
elseif dimensions(1) == 3
    x(3,:) = d1_vec';
    fprintf('First dimension is z \n');    
end

if dimensions(2) == 1
    x(1,:) = d2_vec';
    fprintf('Second dimension is x \n');
elseif dimensions(2) == 2
    x(2,:) = d2_vec';
    fprintf('Second dimension is y \n');
elseif dimensions(2) == 3
    x(3,:) = d2_vec';
    fprintf('Second dimension is z \n');    
end

xd = feval(ds, x);
fprintf('done\n');

h = streamslice(d1_tmp, d2_tmp,reshape(xd(dimensions(1),:),ny,nx), reshape(xd(dimensions(2),:),ny,nx),4,'method','cubic');
set(h,'LineWidth', 0.75)

end