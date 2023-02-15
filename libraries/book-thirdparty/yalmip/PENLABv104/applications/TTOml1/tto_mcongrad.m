function [Akdx, userdata] = sdp_mcongrad(x,Y,k,i,userdata)
% Compute derivatives: d/dx_i A_k(x) based on the data from sdpdata
% in this context it is in fact -F_i of the specific block

  Akdx=[];
  if (k<=0 || k>userdata.Na)
    return;
  end
  if (i<=0 || i>userdata.Nx)
    return;
  end

  Akdx=-userdata.A{1,i+1};

