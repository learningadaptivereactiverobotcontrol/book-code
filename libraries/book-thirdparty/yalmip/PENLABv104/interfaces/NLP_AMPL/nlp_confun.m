function [g,userdata] = nlp_confun(x,Y,userdata)
% get bits from AMPLF (old) interface
% merge inequal & equal, get rid of box
% userdata is expected to keep 'ps' structure from AMPL

%
% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  [f, gx, hx] = amplf(x,0);
  g = [gx(userdata.N_BOUNDS+1:end); hx];

