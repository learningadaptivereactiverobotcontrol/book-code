function [xd] = locally_rotate_2d(xd, h_s, h_x, phi_c)
nX  = size(xd,2);
dim = size(xd,1);
for j=1:nX
    % Local Rotation
    % M(x) = [cos(phi(x)) -sin(phi(x));
    %         sin(phi(x)) cos(phi(x))];
    M_r = zeros(dim,dim);
    M_r(1,1) = cos(h_x(j)*h_s(j)*phi_c);M_r(1,2) = -sin(h_x(j)*h_s(j)*phi_c);
    M_r(2,1) = sin(h_x(j)*h_s(j)*phi_c);M_r(2,2) = cos(h_x(j)*h_s(j)*phi_c);
    xd(:,j) = M_r * xd(:,j);
end
