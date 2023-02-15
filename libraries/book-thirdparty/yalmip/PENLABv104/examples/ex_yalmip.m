% EX_YALMIP shows how to use YALMIP to define an LMI or BMI problem
% and then export the problem to and solve it by PENLAB
%
% See also pen2bmi, yalmip2bmi, bmi_define

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

% YALMIP commands
% LMI example
%    A = [-1 2;-3 -4];
%    P = sdpvar(2,2);
%    F = [P >= eye(2), A'*P+P*A <= 0];
%    pen = export(F,trace(P),sdpsettings('solver','penbmi'),[],[],1);

% BMI example
  A = [-1 2;1 -3]; B=[-1;2];
  P = sdpvar(2,2); K = sdpvar(1,2);
  F = [(A+B*K)'*P+P*(A+B*K) < -eye(2); P>eye(2)];
  pen=export(F,trace(P),sdpsettings('solver','penbmi'),[],[],1);

%PENLAB commands
  bmi = yalmip2bmi(pen);
  penm = bmi_define(bmi);
  prob = penlab(penm);
  solve(prob);
  prob.x

