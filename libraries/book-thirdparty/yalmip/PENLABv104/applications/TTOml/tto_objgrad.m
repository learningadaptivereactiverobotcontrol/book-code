function [df, userdata]=sdp_objgrad(x,Y,userdata)
% return function value of the objective
% return Nx x 1
  df = userdata.c;

