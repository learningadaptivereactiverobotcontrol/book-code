function [nFlag,rResults]=eqconstr_min(obj)
% equality constrained minimization
%
% obj() etc. should work with parameters x and UEQ (equality lagrangian 
% multipliers). If UEQ!=0, the function returns lagrangian 
% (e.i., augmented lagrangian for inequalities + lagrangian for equalities)
% as the first return value and function value/gradient of equality constraints
% as the second one; obj_hes() has just one return argument (hessian)
% expects an open nl-file?, N, N_EQUAL, ... global variables
%
% the equality constrainted problem
% update different LS procedures


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  starttime=cputime;
  % reset default parameters if necessary
  MAX_MITER = obj.allopts.max_inner_iter;
  ALPHA = obj.allopts.inner_stop_limit;
  TOL_DIR = obj.allopts.eq_dir_stop_limit;
  solver = obj.allopts.eq_solver;
  linesearch = obj.allopts.eq_linesearch;
  solver_warn_max = obj.allopts.eq_solver_warn_max;
  ls_short_max = obj.allopts.ls_short_max;
  recover_strategy = obj.allopts.min_recover_strategy;
  recover_max = obj.allopts.min_recover_max;
  
  miter=0;
  solver_warn=0;
  ls_short=0;
  recover_no=0;
  tol_dir_orig=TOL_DIR;

  %x=x0;       starting point is the current point
  %ueq = ueq0;
  %[fx,eqx]=fnc.obj(x,ueq);
  %[gradx, gradeqx]=fnc.obj_grad(x,ueq);
  obj.eval_alx();
  obj.eval_aldx();
  fx_start = obj.ALx;
  rNormG = norm(obj.ALdx);
  rNormG_start = rNormG;
  infeas_eq = norm(obj.eqx);
  infeas_eq_start = infeas_eq;

  obj.print(3,Inf,'object(x_%3i) = %24.16E',miter,obj.ALx);
  obj.print(3,Inf,'||grad(x)||_2 = %24.16E',rNormG);
  obj.print(3,Inf,'infeas_eq     = %24.16E',infeas_eq);
  obj.print(3,4,'     ----');
  obj.print(4,Inf,'        --- start of inner iter ---');
  obj.print(4,Inf,' ');

  nFlag=1; % max it limit reached
  rResults=[];
  obj.initer_last=0;
  obj.lsiter_last=0;
  obj.stats_time_fact_last=0;

  % init LS
  if (linesearch==9)
    error('not adjusted for PenLab yet');
    %lseq_inoc(fnc);
    obj.lseq_inoc();
  end

    while (miter < MAX_MITER)

      %hessx=fnc.obj_hess(x,ueq);
      obj.eval_alddx();
      %hessx=obj.ALddx;

      %%%%%%%% Newton solver %%%%%%%%%
      switch solver
      case 0  % LDL factorization
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_ldl(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx);
      case 1  % LU factorization
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_lu(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx);
      case 2  % MA57 factorization
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_ma57(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx);
      case 3  % Luksan 3
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_luksan3(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx,TOL_DIR,20+obj.Nx+obj.NYnnz+obj.Neq);
      case 4  % Luksan 4
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_luksan4(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx,TOL_DIR,20+2*(obj.Nx+obj.NYnnz-obj.Neq));
      case 5  % Schur complement + CGM
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_schurcgm(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx,TOL_DIR,20+2*obj.Neq);
      case 6  % Projected CGM onto nullspace (implicit)
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_projcgm(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx,TOL_DIR,20+2*(obj.Nx+obj.NYnnz-obj.Neq));
      case 7  % Projected CGM onto nullspace (implicit, LDL)
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_projcgm2(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx,TOL_DIR,20+2*(obj.Nx+obj.NYnnz-obj.Neq));
      case 8  % CGM on the nullspace (implicit Z based on the structure of FMO)
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_nscgm(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx,TOL_DIR,20+2*(obj.Nx+obj.NYnnz-obj.Neq));
      case 103  % Luksan 3 with a special stop crit
        stop_test=@obj.lseq_inoc;
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_luksan3e(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx,stop_test,20+obj.Nx+obj.NYnnz+obj.Neq);
      case 104  % Luksan 4 & LS stop crit
        stop_test=@obj.lseq_inoc;
        [x_dir,ueq_dir,nFlagSol] = obj.solvekkt_luksan4e(obj.ALddx,obj.eqdx,-obj.ALdx,-obj.eqx,stop_test,20+2*(obj.Nx+obj.NYnnz-obj.Neq));
      %case   %
      otherwise
        obj.print(1,Inf,'ERR @ eqconstr_min(): Sorry, no such solver, terminating...');
        nFlag=100;
        break;
      end

      if (nFlagSol>0)  % solver fatal error
        nFlag=2; % solver failed, cannot continue
	    obj.print(3,Inf,'FAILURE: KKT solver, cannot continue (%i)',nFlagSol);
        break;
      elseif (nFlagSol<0)  % solver error, however, hopefully recoverable
        solver_warn = solver_warn+1;
        if (solver_warn>solver_warn_max)
          nFlag=2;
          obj.print(3,Inf,'FAILURE: KKT solver, cannot continue (flag %i or similar happened %i times)',nFlagSol,solver_warn);
          break;
        else
          obj.print(4,Inf,'Step accepted as a trial one (remaining: %d).\n',solver_warn_max-solver_warn);
        end
      else % solver OK
        solver_warn=0;
      end

      %%%%%%%% linesearch %%%%%%%%%
      % get the maximal step length
      %rMaxStep=obj.get_maxstep(x_dir);   % TODO ... no barriers so no problem
      rMaxStep=1;
      nMaxSteps=ceil(-log(rMaxStep)/log(2));
      obj.print(5,Inf,'LSEQ: max step %.1e, %i steps',rMaxStep,nMaxSteps);

      switch linesearch
      case 0  % Do nothing, leave original data
	 % such as x, grad_x (useful for TR, ...)
      case 1  % Do full steps, no linesearch at all
        [rRelStep, nFlagLS] = obj.lseq_fullstep(x_dir,ueq_dir);
      %case 2  % Armijo linesearch
      %  [rRelStep, nFlagLS] = ls_armijo(dir);
      case 3  % Pennlp/Pennon equality linesearch
        [rRelStep, nFlagLS] = obj.lseq_pen(x_dir,ueq_dir,miter);
      case 4  % Filter linesearch, first version, Biegler/Wachter
        [rRelStep, nFlagLS] = obj.lseq_filter(x_dir,ueq_dir,miter);
      case 5  % Nocedal Knitro LS, simple, first version
        [rRelStep, nFlagLS] = obj.lseq_noc(x_dir,ueq_dir,obj.ALddx*x_dir,miter);
      case 6  % Nocedal Flexible Merit LS
        [rRelStep, nFlagLS] = obj.lseq_flex(x_dir,ueq_dir,obj.ALddx*x_dir,miter);
      case 7  % Nocedal Knitro LS, simple, 2nd version
        [rRelStep, nFlagLS] = obj.lseq_noc2(x_dir,ueq_dir,obj.ALddx*x_dir,miter);
      case 8  % Nocedal Knitro LS, simple, 3nd version
        [rRelStep, nFlagLS] = obj.lseq_noc3(x_dir,ueq_dir,obj.ALddx*x_dir,miter);
      case 9  % Nocedal Knitro inexact SQP LS
        [nFlagLS, rRelStep] = obj.lseq_inoc(x_dir,ueq_dir,obj.ALddx*x_dir);
      %case   df%
      otherwise
        obj.print(1,Inf,'ERR @ eqconstr_min(): Sorry, no such LS, terminating...');
        nFlag=100;
        break;
      end

      if (nFlagLS>0 && solver_warn>0)
        nFlag=2; % LS failed because of solver, cannot continue
        obj.print(3,Inf,'FAILURE: Linesearch (because of the KKT solver), cannot continue');
        %break;
      elseif (nFlagLS>0)
        nFlag=3; % LS failed, cannot continue
        obj.print(3,Inf,'FAILURE: Linesearch, cannot continue');
        %break;
      elseif (nFlagLS<0)
        ls_short = ls_short+1;
        if (ls_short > ls_short_max)
          nFlag=3; % LS failed, cannot continue
          obj.print(3,Inf,'FAILURE: Linesearch (flag %d, wawrnings %d times), cannot continue',nFlagLS,ls_short);
          %break;
        else
          obj.print(4,Inf,'Short LS step accepted (remaining %d)',ls_short_max-ls_short);
        end
      else  % LS step OK
        ls_short=0;
      end

      % return original settings
      if (recover_no>0)
        %TOL_DIR = check_opt(PEN_OPT, 'eq_dir_stop_limit', TOL_DIR_def);
        TOL_DIR=tol_dir_orig/10;
        solver = obj.allopts.eq_solver;
        if (nFlag<=0)  % strategy was successful
          recover_no=0;
	    end
      end

      % recover from LS/solver failure?
      if (nFlag>1)  % not for iterarion limit
        switch recover_strategy
        case 0  % do nothing - fail
          break;

        case 1  % increase precision of the iterative solver or switch to LDL
          if (recover_no>=recover_max)  % failed to recover
            obj.print(3,Inf,'FAILURE: recovery strategy - no attemps left %i out of %i',recover_no, recover_max);
            break;
          end
          if (solver<3)  % applicable only for iterative methods
            break;  
          end
          if (solver_warn>0 || recover_no>=recover_max-1)   % solver in troubles -> do not increase precision -> switch solver
            obj.print(3,Inf,'\nRecover strategy - switching from iterative methods to LDL');
            solver=0;
          else
            TOL_DIR=TOL_DIR/(100^(recover_no+1));
          end

        case 2  % switch to LDL
          if (recover_no>0)
            obj.print(3,Inf,'FAILURE: recover strategy to switch to LDL didn''t help');
            break;
          end
          if (solver==0)
            break;
          end
          solver=0;

        otherwise
          obj.print(3,Inf,'WARNING @ eqconstr_min(): Sorry, no such min_recover_strategy');
          break;
        end
        % try to recover with the new settings
        recover_no = recover_no+1;
        obj.print(3,Inf,'\nTry to recover from failure (nFlag=%i) using strategy %i',nFlag,recover_strategy);
        nFlag=0;
        continue;
      end


      %%%%%%%% %%%%%%%%%
      rNormG = norm(obj.ALdx);
      infeas_eq = norm(obj.eqx);
      miter=miter+1;

      obj.print(4,Inf,' ');
      obj.print(3,Inf,'object(x_%3i) = %24.16E',miter,obj.ALx);
      obj.print(3,Inf,'||grad(x)||_2 = %24.16E',rNormG);
      obj.print(3,Inf,'infeas_eq     = %24.16E',infeas_eq);
      obj.print(3,4,'     ----');
      obj.print(4,Inf,'        --- end of %3i in. iter ---\n',miter);

      %%%%%%%% stopping criterion %%%%%%%%%
      % just make it simple at the begining
      if (rNormG < ALPHA && infeas_eq < ALPHA)
        nFlag=0;
        break;
      end

    end % of while


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

