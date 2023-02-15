function [nFlag,rResults]=unconstr_min(obj)
% unconstrained minimization
% returns in rResults=[aug_lagr(xBest),norm(grad(xBest)), norm(grad(x_0))]
%
% obj() etc. should work with one parameter x and return first parameter value/grad/hess
% expects an open nl-file?, N, N_EQUAL, ... global variables
%
% in the structure called 'fnc' there should be obj, obj_grad, obj_hess - functions to evaluate
% the equality constrainted problem

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  starttime=cputime;
  % reset default parameters if necessary
  MAX_MITER = obj.allopts.max_inner_iter;
  ALPHA = obj.allopts.inner_stop_limit;
  TOL_DIR = obj.allopts.unc_dir_stop_limit;
  solver = obj.allopts.unc_solver;
  linesearch = obj.allopts.unc_linesearch;

  miter=0;

  % x0 = xall as a starting point
  %fx=fnc.obj(x);
  %gradx=fnc.obj_grad(x);
  obj.eval_alx();
  obj.eval_aldx();
  fx_start = obj.ALx;
  rNormG = norm(obj.ALdx);
  rNormG_start = rNormG;

  obj.print(3,Inf,'object(x_%3i) = %24.16E',miter,obj.ALx);
  obj.print(3,Inf,'||grad(x)||_2 = %24.16E',rNormG);
  obj.print(4,Inf,'        --- start of inner iter ---');
  obj.print(4,Inf,' ');
  obj.print(3,4,'     ----');

  nFlag=1; % max it limit reached
  rResults=[];
  obj.initer_last=0;
  obj.lsiter_last=0;
  obj.stats_time_fact_last=0;

    while (miter < MAX_MITER)

      %hessx=fnc.obj_hess(x);
      obj.eval_alddx();

      %%%%%%%% Newton solver %%%%%%%%%
      switch solver
      case 0  % (Modified) Cholesky factorization
	[dir,nFlagSol] = obj.solve_chol(obj.ALddx,-obj.ALdx);
      %case 1  % CG+??
      %case   %
      %case   %
      otherwise
        obj.print(1,Inf,'unconstr_min() error: Sorry, no such solver, terminating...');
	nFlag=100;
	break;
      end

      if (nFlagSol>0)
        nFlag=2; % solver failed, cannot continue
	obj.print(3,Inf,'FAILURE: Newton solver (%i) cannot continue (flag %i)',solver,nFlagSol);
        break;
      end

      %%%%%%%% linesearch %%%%%%%%%
      switch linesearch
      case 0  % Do nothing, leave original data
	 % such as x, grad_x (useful for TR, ...)
        nFlagLS = 0;
      case 1  % Do full steps, no linesearch at all
        [rRelStep, nFlagLS] = obj.ls_fullstep(dir);
      case 2  % Armijo linesearch
        [rRelStep, nFlagLS] = obj.ls_armijo(dir);
      case 3  % Pennlp/Pennon ("els) linesearch
        [rRelStep, nFlagLS] = obj.ls_pennon(dir);
      %case   %
      otherwise
        obj.print(1,Inf,'unconstr_min() error: Sorry, no such LS, terminating...');
	nFlag=100;
	break;
      end

      if (nFlagLS>0)
        nFlag=3; % LS failed, cannot continue
	obj.print(3,Inf,'FAILURE: Linesearch (%i) cannot continue (flag %i)',linesearch,nFlagLS);
        break;
      end

      %%%%%%%% %%%%%%%%%
      rNormG = norm(obj.ALdx);
      miter=miter+1;

      obj.print(4,Inf,' ');
      obj.print(3,Inf,'object(x_%3i) = %24.16E',miter,obj.ALx);
      obj.print(3,Inf,'||grad(x)||_2 = %24.16E',rNormG);
      obj.print(4,Inf,'        --- end of %3i in. iter ---\n',miter);
      obj.print(3,4,'     ----');

      %%%%%%%% stopping criterion %%%%%%%%%
      % just make it simple at the begining
      if (rNormG < ALPHA)
        nFlag=0;
	break;
      end

    end % of while

  if (nFlag==1)
    obj.print(3,Inf,'FAILURE: Unconstr minimization max iter (%i) reached.',miter);
  elseif (nFlag==0)
    obj.print(4,Inf,'Unconstr min OK');
  end

  rResults=[obj.ALx,rNormG, rNormG_start];

  % update stats
  obj.initer = obj.initer+obj.initer_last;
  obj.lsiter = obj.lsiter+obj.lsiter_last;
  obj.miter=obj.miter+miter;
  obj.miter_last=miter;
  obj.stats_time_miter_last = cputime - starttime;
  obj.stats_time_miters = obj.stats_time_miters + obj.stats_time_miter_last;
  obj.stats_time_fact = obj.stats_time_fact + obj.stats_time_fact_last;

  return;

