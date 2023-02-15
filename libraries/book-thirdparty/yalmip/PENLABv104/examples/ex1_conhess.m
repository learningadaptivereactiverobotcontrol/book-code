function [ddgk, userdata] = ex1_conhess(x,Y,k,userdata)
% Hessians of the constraints, Example 1

  ddgk = 2.*eye(3,3);

