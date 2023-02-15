function [Akdx, userdata] = sdp_mcongrad(x,Y,k,i,userdata)
% Compute derivatives: d/dx_i A_k(x) based on the data from sdpdata
% in this context it is in fact -F_i of the specific block

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  Akdx=[];
  if (k<=0 || k>userdata.Na)
    return;
  end
  if (i<=0 || i>userdata.Nx)
    return;
  end

  Akdx=-userdata.A{k,i+1};

