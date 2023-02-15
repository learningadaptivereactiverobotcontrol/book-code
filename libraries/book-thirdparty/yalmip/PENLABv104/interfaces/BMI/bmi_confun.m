function [g,userdata] = bmi_confun(x,Y,userdata)
% BMI_CONFUN returns values of all function constraints g(x) at once 
% (vector Ng x 1) they will be considered in constraints: lbg <= g(x) <= ubg
% where lbg, ubg are bounds given in initialization

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  g=userdata.B*x;

