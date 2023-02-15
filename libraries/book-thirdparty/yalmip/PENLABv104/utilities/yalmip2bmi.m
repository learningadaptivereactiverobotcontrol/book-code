function [bmidata]=yalmip2bmi(yalpen, name)
% YALMIP2BMI reads the output of YALMIP "export" command and
% converts it into a structure accepted by 
% PenLab via BMI or PMI modules, for details see manual or
%    modules/BMI/bmi_define.m.
%
% See also pen2bmi, ex_yalmip, bmi_define
%
% Example of using YALMIP2BMI:
%  %YALMIP commands
%    A = [-1 2;-3 -4]; B=-[1;1];
%    P = sdpvar(2,2); K = sdpvar(1,2);
%    F = [(A+B*K)'*P+P*(A+B*K) < -eye(2); P>eye(2)]
%    yalpen=export(F,trace(P),sdpsettings('solver','penbmi'),[],[],1);
%  %PENLAB commands
%    bmi=yalmip2bmi(yalpen);
%    penm = bmi_define(bmi);
%    prob = penlab(penm);
%    solve(prob);

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013



  if (nargin<=1)
    name='BMI2 from convertor pen2bmi2()';
  end

  bmidata=pen2bmi(yalpen.penstruct, name);