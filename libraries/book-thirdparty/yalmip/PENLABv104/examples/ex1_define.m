function [penm] = ex1_define()
% define Example 1 as is necessary to handle by PenLab
% call:
%   penm=ex1_define();    % to define structure 
%   prob=penlab(penm);    % to convert the structure and initialize the problem
%   prob.opts....=...;    % to change option settings if desired
%   prob.solve();         % to start the solver
%   prob.x                % to retrieve the final point (solution)

  penm = [];
  % [optional] set problem name/comment (for log files)
  penm.probname = 'examples/ex1';
  penm.comment = 'Source: user external definition of functions';

  penm.Nx = 3;
  penm.lbx = zeros(3,1);   % allow 0 as well

  penm.NgNLN = 1;
  penm.NgLIN = 1;
  penm.lbg = [4, 24];   % would column vector work?
  penm.ubg = [Inf, 24];

  % possible even as anonymous?? Try it
  penm.objfun = @ex1_objfun;
  penm.confun = @ex1_confun;
  penm.objgrad = @ex1_objgrad;
  penm.congrad = @ex1_congrad;
  penm.objhess = @ex1_objhess;
  penm.conhess = @ex1_conhess;
  %penm.lagrhess = @ex1_lagrhess;



