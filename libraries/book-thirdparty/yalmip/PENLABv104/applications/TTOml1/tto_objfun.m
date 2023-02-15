function [f,userdata] = sdp_objfun(x,Y,userdata)

  f = userdata.c'*x;

