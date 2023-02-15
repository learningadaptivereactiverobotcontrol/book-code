function [B] = findDampingBasis(xd)
 y1 = 1;
 y2 = -xd(1)/xd(2);
 y = [y1;y2];
 B = [xd./norm(xd), y./norm(y)];
end
