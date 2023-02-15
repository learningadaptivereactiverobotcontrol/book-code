function [dg, userdata]=nlp_congrad(x,Y,userdata)
% get bits from AMPLF (old) interface

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  [fdx, gdx, hdx] = amplf(x,1);
  dg = [gdx(1:end, userdata.N_BOUNDS+1:end), hdx];

