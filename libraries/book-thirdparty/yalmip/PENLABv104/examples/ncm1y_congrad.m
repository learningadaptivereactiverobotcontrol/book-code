function [dg, userdata]=ncm1y_congrad(x,Y,userdata)
% returns all constraints at once
% vector Ng x Nx --> 1xNYnnz TODO no ... transpose! (Nx+NYnnz) x Ng

  dg=[1;0;1;0;1];

