function [df, userdata]=ncm1a_objgrad(x,Y,userdata)
% return gradient of the objective w.r.t. all variables (even matrix)
% return Nx x 1

  % hard-code me for the moment
  %YH=Y-userdata;
  %df = [2*YH(1,1); 4*YH(2,1); 2*YH(2,2); 4*YH(3,2); 2*YH(3,3)];

  H=userdata.H;
  df=[ 2*(x(1)-H(1,1)); 
       4*(x(2)-H(2,1)); 
       2*(x(3)-H(2,2)); 
       4*(x(4)-H(3,2));
       2*(x(5)-H(3,3)) ];

  %df=[ 2*(x(1)-H(1,1)); 
  %     2*(x(2)-H(2,1)); 
  %     2*(x(3)-H(2,2)); 
  %     2*(x(4)-H(3,2));
  %     2*(x(5)-H(3,3)) ];

