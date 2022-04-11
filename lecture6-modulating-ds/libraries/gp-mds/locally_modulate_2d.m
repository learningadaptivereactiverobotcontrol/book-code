function [xd] = locally_modulate_2d(xd, h_x, A)
nX  = size(xd,2);
dim = size(xd,1);
for j=1:nX
    % Local Modulation by Random Matrix M(x) = (1 - h(x))*I + h(x)*A
    xd(:,j) = ((1-h_x(j))*eye(dim,dim) + h_x(j)*A) * xd(:,j);
end
