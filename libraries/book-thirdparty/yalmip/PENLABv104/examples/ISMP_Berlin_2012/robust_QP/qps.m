function [penm] = qps(A,b)
% 
  n = length(A);
  penm = [];

  % [optional] set problem name/comment (for log files)
  penm.probname = 'Quadratic programming on unit simplex';
  penm.comment  = 'using anonymous functions';

  penm.Nx = n;
  penm.lbx = zeros(n,1);

  penm.NgLIN = 1;
  penm.lbg = [1];
  penm.ubg = [1];   

  penm.objfun  = @(x,Y,userdata) deal(x'*A*x-b'*x, userdata);
  penm.objgrad = @(x,Y,userdata) deal(2*A*x-b, userdata);  
  penm.objhess = @(x,Y,userdata) deal(2*A, userdata);
  
  penm.confun  = @(x,Y,userdata) deal(sum(x), userdata);
  penm.congrad = @(x,Y,userdata) deal(ones(n,1), userdata);




