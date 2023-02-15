function [dg, userdata]=sdp_congrad(x,Y,userdata)
% returns all inequalities at once
% matrix Nx x Ng

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  if (userdata.Ng>0)
    dg=userdata.B';
  else
    dg=[];
  end

