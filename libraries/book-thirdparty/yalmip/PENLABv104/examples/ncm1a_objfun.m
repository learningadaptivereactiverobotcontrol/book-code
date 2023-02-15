function [f,userdata] = ncm1a_objfun(x,Y,userdata)
  % f = sum_ij  (x_ij - h_ij)^2
  % matrix H is stored in userdata

  % I could build Y and do this, but let's write it directly
  %YH = Y-userdata;
  %f = YH(:)'*YH(:);

  H=userdata.H;
  f=(x(1)-H(1,1))^2 + 2*(x(2)-H(2,1))^2 + (x(3)-H(2,2))^2 + 2*(x(4)-H(3,2))^2 + (x(5)-H(3,3))^2;
  %f=(x(1)-H(1,1))^2 + (x(2)-H(2,1))^2 + (x(3)-H(2,2))^2 + (x(4)-H(3,2))^2 + (x(5)-H(3,3))^2;

