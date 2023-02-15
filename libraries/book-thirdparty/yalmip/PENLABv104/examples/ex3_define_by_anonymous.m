function [penm] = ex3_define_by_anonymous()
% define Example 3 as is necessary to handle by PenLab
% same as ex3_define() but this time using anonymous functions 
% instead of m-files
% call:
%   penm=ex3_define_by_anonymous();    % to define structure 
%   prob=penlab(penm);    % to convert the structure and initialize the problem
%   prob.opts....=...;    % to change option settings if desired
%   prob.solve();         % to start the solver
%   prob.x                % to retrieve the final point (solution)

  penm = [];

  % [optional] set problem name/comment (for log files)
  penm.probname = 'examples/ex3';
  penm.comment = 'Source: user external definition of functions, using anonymous functions';

  penm.Nx = 2;
  penm.lbx = [-0.5; -Inf];
  penm.ubx = [0.5; 1];

  penm.NgNLN = 2;
  %penm.NgLIN = 0;          % optional, 0 by default
  penm.lbg = [0; 0];
  %penm.ubg = [Inf, Inf];   % optional, +Inf for upper bounds by default

  % define all call-backs as anonymous functions
  % The problem is that all the functions need to return 2 arguments
  % (2nd one is userdata and it is compulsory at the moment even if
  % it is not used). Therefore we need to use deal() Matlab built-in
  % routine to do that
  penm.objfun = @(x,Y,userdata) deal(100*(x(2) - x(1)^2)^2 + (1 - x(1))^2, userdata);
  penm.confun = @(x,Y,userdata) deal([x(1)^2 + x(2); x(1) + x(2)^2], userdata);
  penm.objgrad = @(x,Y,userdata) deal([-400*x(1)*(x(2)-x(1)^2) - 2*(1-x(1)); 200*(x(2)-x(1)^2)], userdata);
  penm.congrad = @(x,Y,userdata) deal([2*x(1), 1; 1, 2*x(2)], userdata);
  penm.lagrhess = @(x,Y,v,userdata) deal([1200*x(1)^2-400*x(2)+2, -400*x(1); -400*x(1), 200] + v(1)*[2, 0; 0, 0] + v(2)*[0, 0; 0, 2], userdata);

  % [optional] set starting point
  %penm.x = zeros(2,1);
  penm.x = [-2; 1];        % suggested starting point


