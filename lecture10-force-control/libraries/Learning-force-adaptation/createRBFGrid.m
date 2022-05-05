function rbf_grid = createRBFGrid(grid_center, grid_size, nb_gaussians)
rbf_grid = zeros(prod(nb_gaussians),3);
% Compute kernel position along x and y
x = linspace(-grid_size(1)/2,grid_size(1)/2,nb_gaussians(1));
y = linspace(-grid_size(2)/2,grid_size(2)/2,nb_gaussians(2));
% Generate grid
count = 1;
for k = 1:length(x)
    for m = 1:length(y)
        rbf_grid(count,1) = x(k)+grid_center(1);
        rbf_grid(count,2) = y(m)+grid_center(2);
        count = count+1;
    end
end
end

