function [rRelLength] = get_maxstep(obj,x,dir,varargin)
% Computes the maximal step length at point 'x' in direction 'dir'
% to be still in the inner area of barriered constraints
% At the moment, the only supported constraints in a barrier are
% box constraints. For others it is necessary to change this func.
%
% Function has to be initiated from the outer loop level (where constraints
% are not composed to the augmented Lagrangian yet). To init call:
% get_maxstep(x,[],barind,gx,dgx);
% where 'barind' is a list of indicies of the constraints in the barrier,
% gx, dgx are function and gradient values of inequality constraints
%
% After the initialization it is possible to use the function in
% the inner loop: rMaxStep=get_maxstep(x,dir);
% The step length is limited to maximal length 1.0.
%
% If the function is not initializated or no restrictions are given,
% returns 1.0 always.
%

%
% last update: 10/08/2009
%

  error('not adjusted for PenLab yet.');
  % all considered barriered constraints are of the form: +/-1*x_i + c_j<=0
  % the j-th constraint is: xsgn(j) * x(xidx(j)) + constants(j) <=0
  persistent no_bconst;  % number of these (barriered) constraints 
  persistent constants;  % constants c_j, j=1:no_constr
  persistent xidx;       % there is x(xidx(j)) in the j-th constraint 
  persistent xsgn;       %   ... with sign xsgn(j)

  % distance from the boundary?
  bound_margin = 0.95;

  % initialization
  if (size(varargin,2) > 0)
    rRelLength=1.0;  % doesn't really matter 
    if (size(varargin,2) ~= 3)
      pen_out(5,Inf,'Error@get_maxstep.m: Inconsistant call, user error?');
      return;
    end
    barind=varargin{1};
    gx=varargin{2};
    dgx=varargin{3};

    no_bconst=length(barind);
    if (no_bconst~=0)
      [xidx,col_ignore,xsgn]=find(dgx(:,barind));
      % in each column there should be exactly one entry +1 or -1
      % in the row appropriate to the index of 'x'
      % sorted column-wise
      constants=gx(barind)-xsgn.*x(xidx);
    else
      constants=[];
      xidx=[];
      xsgn=[];
    end
    return;
  end

  % normal usage - no restriction/not init
  if (isempty(no_bconst) || no_bconst==0)
    rRelLength = 1.0;
    return;
  end

  % normal usage - restrictions apply
  sdir=xsgn.*dir(xidx);
  idx=find(sdir > 0);
  rRelLength=min([-bound_margin*(constants(idx) + xsgn(idx).*x(xidx(idx)))./sdir(idx); 1.0]);
  %rRelLength=min(1.0, rRelLength);

  return;

