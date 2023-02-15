function [Akx, userdata] = sdp_mconfun(x,Y,k,userdata)
% evaluate A_k(x) based on sdpdata, k denotes a block number
% sdpdata is a structure as obtained from readsdpa.m
% note that we aim for A_k(x)<=0 thus A_k(x) is 'reversed' than usual:
%   A_k(x) = F_0 - sum x_i*F_i

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  Akx=[];
  if (k<=0 || k>userdata.Na)
    return;
  end

  Akx=userdata.A{k,1};
  for i=1:userdata.Nx
    % won't work if F{} is [] ... <== dims must match
    Akx = Akx - x(i).*userdata.A{k,i+1};
    % or - ?
  end

