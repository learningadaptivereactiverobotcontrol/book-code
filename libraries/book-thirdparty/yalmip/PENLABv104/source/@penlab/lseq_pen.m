function [rRelStep, nFlag]=lseq_pen(obj, dir, udir, miter)
% dir/udir ... new directions for x/equality constraints multipliers
%
% miter = v kolikate jsem velke iteraci, pokud 0 ~ delam prvni, dopad na rNu
% fx/gradx ~ function value/gradient of lagrangian, e.i., eqlty constraints included
%

% TO DO:
% * fx<HUGE_VAL ... check how it works
% * bStop problem... (stop crit)
% * fail-safe by using normal LS
%

%%%%%%%%% Settings %%%%%%%%%%
  rAlphaMin = 1.0e-14;
  rPrecfac=10.0;
  rMacheps=1.0e-16; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  bJustStarted=0;
  if (isempty(obj.ls_rnu) || miter==0)
    obj.ls_rnu=0;
    bJustStarted=1;
  end

  LSEQ_MAX_ITER = obj.allopts.max_lseq_iter;
  xall0 = obj.xall;
  ueq0 = obj.ueq;

%% co s multiplikatory?  global/parametr?
%% Jak to brat defaultne? fx/gradx jsou Lagrangianu s multiplikatory? ano...?

  % find maximal step length
  rStepMax = 1.;

  % note: reduce gradient by eq. constr.
  rPi = dir'*(obj.ALdx - obj.eqdx*obj.ueq);
  rEqNorm0 = obj.eqx'*obj.eqx;
  rU0C0 = obj.ueq'*obj.eqx;
  rUgVh0 = (obj.ueq+udir)'*obj.eqx - 2*rU0C0;  %??

  % new penalty parameter rNu estimate
  rNu_old = obj.ls_rnu;
  if (rEqNorm0 > 1e-12)
    rEtadd = (rPi + rUgVh0) / rEqNorm0;
  else
    rEtadd = 0.;
  end

  if (bJustStarted)
    obj.ls_rnu = abs(rEtadd) + 0.1; % rNu0 = 0.1 !!!
  else
    obj.ls_rnu = max(0.,rEtadd) + 0.1;
  end
  %obj.ls_rnu = max(rNu_old, obj.ls_rnu);

  obj.print(4,Inf,'LSEQ (pen): penalty par. NU=%f', obj.ls_rnu);
      
  % Add new penalty term
  fx = obj.ALx;
  fx0 = fx;            % original name in line_search_eq.c: rFobj_old, fx~rFobj
  meritx0 = fx0 + .5*obj.ls_rnu*rEqNorm0;         % original name: rF0
  rDelal0 = rUgVh0 - obj.ls_rnu * rEqNorm0 + rPi;

  obj.print(4,Inf,'LSEQ (pen): NU: %e %e %f', rDelal0, rEqNorm0, rEtadd);

  rAlphaStep = 2.0*rStepMax;

  ok=0;
  for iter=1:LSEQ_MAX_ITER
    rAlphaStep = rAlphaStep / 2.0;        
        
    % Compute new trial point and evaluate objective and constraint
    % function at that point
    obj.xall = xall0 + rAlphaStep*dir;
    obj.ueq = ueq0 + rAlphaStep*udir;
      
    obj.eval_alx();
    fx=obj.ALx;
    %[fx,eqx]=fnc.obj(x_it,ueq_it);
    rEqNorm=obj.eqx'*obj.eqx;
    meritx = fx + .5*obj.ls_rnu*rEqNorm;

    if (iter==1 && abs((fx - fx0)/(.5*(fx + fx0))) + abs(rEqNorm0 - rEqNorm) < 1.0e-14)
      ok=2;    % in Pennlp: bStop <-- 1; ~ stop crit of unconstr_min <d,g>
      break;
    end

    rUC = obj.ueq'*obj.eqx;
    rUgVh = rUC - rU0C0;

    rZlhs = fx - fx0 + rUgVh - 1.0e-4 * rAlphaStep * (rPi + rUgVh0);
    rZrhs = (rEqNorm0 - rEqNorm) * .5 - rAlphaStep * 1.0e-4 * rEqNorm0;

    if (abs(rZrhs) <= 1.0e-15 && rZlhs <= 1.0e-15 && isfinite(fx)) % && *fX < HUGE_VAL)
      ok=1;
      break;
    end
    if (rZrhs >= 0. && isfinite(fx)) % && *fX < HUGE_VAL)
      ok=1;
      break;
    end
        
    rNu_ls = rZlhs / rZrhs;

    if (iter > 1 && rNu_ls > obj.ls_rnu && isfinite(fx)) % && *fX < HUGE_VAL)
      ok=1;
      break;
    end

    rLhs = meritx - meritx0 - rPrecfac * rMacheps * abs(meritx0);
    rRhs = rAlphaStep * 1.0e-4 * rDelal0;
        
    if (rLhs <= rRhs && isfinite(fx)) % && *fX < HUGE_VAL)
      ok=1;
      break;
    end 

    if (rAlphaStep <= rAlphaMin)   % really????
      ok=1;
      break;
    end 

  end

  % If evrth. OK update ...
  if (ok>0)
    obj.print(3,Inf,'LSEQ (pen): %i steps, rel. width %f',iter,rAlphaStep);
    %x=x_it;
    %ueq=ueq_it;
    obj.eval_aldx();
    %[gradx, gradeqx] = fnc.obj_grad(x, ueq);
    rRelStep=rAlphaStep;
    nFlag=0;
  else
    obj.print(3,Inf,'LSEQ (pen): failed - step too short, max_ls_it (%i)',iter);
    % x/ueq unchanged
    obj.xall=xall0;
    obj.ueq=ueq0;
    obj.eval_alx();
    %[fx,eqx]=fnc.obj(x,ueq);
    rRelStep=0;
    nFlag=3;
  end

  obj.lsiter_last = obj.lsiter_last+iter;

%  if (!ok)
%    for iter=1:4
%      obj.ls_rnu = obj.ls_rnu * 10.0;
%      obj.print(3,Inf,'LSEQ (pen): Nu = %10.2E (short step)', obj.ls_rnu);
%      merit_obj=@(xtmp,ueqtmp) add_penalty_to(fnc.obj,obj.ls_rnu,xtmp,ueqtmp);
       % call ordinary linesearch, do not forget, that now it returns values of the merit function!
%      rRelStepLength = line_search((* objective), (* gradient_obj), gradx, S, X, fX, rStepLength, 1., 1);
%      if ok ... break;
%    end
%  end

  return;


function [phi, constr] = add_penalty_to(objective, rNu, x, ueq)
% suppose [f, constr]=objective(x,ueq), create a new function (merit function)
% such that returns [f+0.5*rNu*constr'*constr]
% need to mimic behaviour as objective but need to return merit function
% usage: merit=@(xtmp,utmp) add_penalty_to(obj, rNu, xtmp, utmp)
% then merit(x,ueq) does its job

  % this would need to be changed, but it is not used anyway
  error('this is not adapted for PenLab');
  [f, constr]=objective(x,ueq);
  phi = f + 0.5*rNu*(constr'*constr);

  return;
  
