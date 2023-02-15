function [g,userdata] = sdp_confun(x,Y,userdata)
% returns all inequalities at once, expect g(x)<=0
% vector Ng x 1

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  if (userdata.Ng>0)
    g=userdata.B*x;
  else
    g=[];
  end

