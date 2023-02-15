function [dg, userdata]=sdp_congrad(x,Y,userdata)
% returns all inequalities at once, expect g(x)<=0
% vector Ng x Nx

dg = [ones(length(x)-1,1);0];

