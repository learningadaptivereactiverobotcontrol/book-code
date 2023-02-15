function [Akx, userdata] = sdp_mconfun(x,Y,k,userdata)
% evaluate A_k(x) based on sdpdata, k denotes a block number
% sdpdata is a structure as obtained from readsdpa.m
% note that we aim for A_k(x)<=0 thus A_k(x) is 'reversed' than usual:
%   A_k(x) = F_0 - sum x_i*F_i

  Akx=[];
  if (k<=0 || k>userdata.Na)
    return;
  end

  Akx=userdata.A{k,1};
  for i=1:userdata.Nx
    % won't work if F{} is [] ... <== dims must match
    Akx = Akx - x(i).*userdata.A{1,i+1};
    % or - ?
  end

