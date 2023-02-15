function [rRelStep, nFlag]=ls_pennon(obj, dir)
% rewritten line_search() @ line_search_els.c @ Pennlp v.2.3 & Pennon v.0.9
%
% changes in 'obj': xall, ALx, ALdx

%%%%%%%%% Settings %%%%%%%%%%
  XTOL = 1.0E-1;
  ETA = 0.9;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% d = dir ... computed search direction
  
  MAX_LS_ITER = obj.allopts.max_ls_iter;

  xall0=obj.xall;   % original point
  % whenever I want to evaluate at a new point, xall<-x_it
  fx=obj.ALx;

  % double pi0, pi, mu, divs, mu1s, mu2s, eff, kappa, fact, q;
  mu1 = 1.; mu2 = 1.; sml=0.5;   %%% fbest, f, f0, f1, f2, 
  alp0 = 0; alp1 = 0; alp2 = 0; alps = 0; alpm = 0; alpd = 0; omega1 = 0; omega2 = 0;
  stpmin = 0; stpmax = 1.0E20; alp = 1.; denom = 0.;
  FTOL = 1-ETA;
  
  % int 
  lsiter=0;
  nEff = 0; iterations = 0;
  k = 0; ksh = 0; ier = 0;
  bLong = 0; bSafeguard = 0;
  
  bLinesearch = 1;
  
  rRelStep=0.; nFlag=99;

  %if(Uequal)
  %  if(Uequal->dim) {
  %    stpmax=1.0;
  %  }

 
  pi=dir'*obj.ALdx;
  % computation of the rest of the gradient of our merit function
  %if(Uequal) {
    % v Src 0.9 je pouze:  (ale to je temer jiste blbe
  %  pi -= rNu * rEqNorm;
    % misto nasledujiciho z Pennlp 2.3(fi)
  %  spm3v_mlt( (SPMAT3 *) gradeq, Cequal, v_tmp_short );
  %  rTmp = in_prod(v_tmp_short, d);
  %  pi += rNu*rTmp;
  %  printf("pi: %e, rTmp: %e\n", pi, rTmp);
  %}
  
  %if(Uequal) {
  %  v_copy(Uequal, u_it);
  %  for(i=0; i<Uequal->dim; i++)
  %    u_d->ve[i] = d->ve[X->dim+i];
  %    // smer pro multiplikatory je "schovany" v d za x, ale dim je jen pro x
  %}

  %if(Uequal) {
  %  spm3v_mlt( (SPMAT3 *) gradeq, u_d, v_tmp_short );
  %  rTmp = in_prod(v_tmp_short, d);
  %  pi += rTmp;
  %  printf("pi2: %e, dAv: %e", pi, rTmp);
  %}

  % Is it really descent 
  if (pi > 0)
    obj.print(3,Inf,'LS (pen): No Descent'); 
    
    bLinesearch = 0;
    rRelStep=0;
    nFlag=1;
    return;
  elseif (abs(pi) < 1.0E-10)
    alp = 1.0;
    %obj.xall = xall0 + alp*dir;   % not needed here
    %if (Uequal)
    %  UEQ = u_it + alp*u_dir;
    %end
    
    %while((*fx =  (* objective) (x_it)) >= HUGE_VAL) {
    %  alp = alp/2;
    %  x_it=x+alp*dir;
    %}
    
    if (abs(pi) < 1.0E-15) 
      obj.ls_small_cnt=obj.ls_small_cnt+1;
    else
      obj.ls_small_cnt=0;
    end

    if (obj.ls_small_cnt > 2)
      % x remains unchanged --> nothing todo
      % but objective() could be called elsewhere then at x  
         % doesn't seem so at this version (JF 4/11/11)
      %fx = fnc.obj(x);
      rRelStep=0.;
      nFlag=2;
      obj.print(3,Inf,'LS (pen):  %.4e (almost not descent), no step', pi); 
    else        % still acceptable
      obj.xall = xall0 + alp*dir;
      %x=x_it;
      obj.eval_alx();
      obj.eval_aldx();
      %fx = fnc.obj(x);
      %gradx = fnc.obj_grad(x);
      obj.print(3,Inf,'LS (pen):  %.4e (small full step)', pi); 
      rRelStep=1.;
      nFlag=0;   
      lsiter=lsiter+1;
    end

    return;
  %else
  %  obj.print(3,Inf,'LS (pen):  %.4e', pi);   %%% ?
  end
  
  obj.ls_small_cnt=0;

  %*****************************************************
  %* Initialize line search ... *
  %*****************************************************
  
  if (FTOL <= 0)
    mu1s = 1./3.;
    mu2s = 2./3.;
    eff = 4./9.;
  else
    mu1s = min(FTOL,1/3);
    mu2s = 1-mu1s;
    eff = 2*mu1s*mu2s;
  end
  
  if (XTOL >= 1.) 
    kappa = 2.;
  else 
    kappa = 1. / min(1.,1.-XTOL);
  end
  
  f0 = obj.ALx; f1 = obj.ALx; f2 = obj.ALx; fbest = obj.ALx;
  pi0 = pi;
  
  %*****************************************************
  %* actual line search iteration ... *
  %*****************************************************
  
  nFlag=3;   % max iter
  % for equalities should be:  iterations < LS_MAX_ITER && (< LS_MAX_EQ_ITER if equal)
  for iterations=1:MAX_LS_ITER
    ier = 0;
    % get new function value
    %//printf(" * %f", alp); 

    %x_it = x+alp*dir;
    obj.xall = xall0+alp*dir;
    %if(Uequal)
    %  UEQ = u_it + alp*u_dir; 
    %end
    
    %f = fnc.obj(x_it);
    obj.eval_alx();
    f = obj.ALx;
    nEff = 0;
    
    %*****************************************************
    %* get new trial step alp 
    %*****************************************************
    
    % count requested function evaluations
    k=k+1;
    
    % check correctness of step
    % note that alp1>=alp2 unless alp1=0
    if (alp < stpmin || alp <= alp2)
      % new step size too small
      obj.print(4,Inf,'LS (pen): ERROR: new step size too small');
      nEff = 0;
      ier = 1;
      break;
    elseif (alp > stpmax || (alp >= alp1 && alp1 > 0))
      % new step size too large
      obj.print(4,Inf,'LS (pen): ERROR: new step size too large');
      nEff = 0;
      ier = 1;
      break;
    end
    
    % update best point
    if (f < fbest || (f == fbest && f == f0))
      fbest=f;
      if (f == f0 && (ksh == 0 || alp < alps))
        % store smallest step with f=f0 for possible restauration
        alps = alp;
      end
      if (f == f0 && (ksh == 0 || alp > alpm))
        % store smallest step with f=f0 for possible restauration
        alpm = alp;
      end
      if (f == f0)
        ksh=ksh+1;
      end
    end
    
    % get parabolic minimizer alp0
    if (alp1*alp2 > 0)
      alp0 = alp1;    % default if minimizer does not exist
      denom = (f-f1)/(alp-alp1)-(f-f2)/(alp-alp2);
      if (denom > 0)
        alp0 = 0.5*(alp1+alp2-(f1-f2)/denom);
      end
    end
    
    % compute Goldstein quotient
    mu = (f-f0)/(alp*pi0);
    if (f == f0 && f2 == f0)
      % check whether f=f0 due to very short step
      if (f1 < f0 || ksh <= 6)
        % assume very short step
        mu = 1.;
      end
    end
    
    % update bracket, with safeguard when f=f0
    bLong = (mu < mu1s); %%% ??? || f >= HUGE_VAL/*_isnan(f)*/);
    if (bLong)
      % long step, overwrite alp1
      alp1 = alp;   
      f1 = f;
      mu1 = mu;
      if (k == 1 && mu > 0.5)
        mu1 = 0;
      end
      omega1 = 2*abs(1.-mu1)/alp1;
    else
      % good step or short step, overwrite alp2
      alp2 = alp;   
      f2 = f;
      mu2 = mu;
      omega2 = 2*abs(1.-mu2)/alp2;
    end
    
    if (f2 == f0 && f1 >= f0 && k >= 2)
      % check whether f=f0 steps should be considered long
      % ksh=7 indicates f=f0 for all large alp
      % f1=f0 && alp1=alp2*2 indicates f2=f0 by chance
      if (ksh == 7 || (ksh == 1 && f1 > f0 && alp1 == alp2*1.1))
        ksh=9999;      % ends f=f0 tests
        % restore alp1=alps, alp2=0
        alp1 = alps;
        f1 = f0;
        mu1 = 0;
        omega1 = 0;
        alp2 = 0;
        f2 = f0;
        mu2 = 1;
        omega2 = 0;
      end
    end
    
    %**********************************
    %* convergence tests              * 
    %**********************************
    
    %if (f0 >= HUGE_VAL && f < f0)
      % f0=infinity and f is finite
    %  obj.print(4,Inf,'WARNING: infinite f0 improved to finite');
    %  nEff = 2;
    %  ier = 0;
    %  break;
    %end
    
    if (bLong)
      if (alp1 == stpmin && alp1 ~= 0.)
        % stpmin too large
        obj.print(3,Inf,'WARNING: end of line search forced by stpmin');
        nEff = 2;
        ier = 0;
        break;
      end     
    else
      % now the sufficient decrease condition holds
      if (0)   %/*_isnan(f1)*/
        if (k >= 3 && alp == alp2 && alp1 <= kappa*alp2)
          % step close to a NaN point !!!!!!
          nEff = 1;
          ier = 0;
          break;
        end
      end
      
      if (alp2*mu2*max(omega1,omega2) >= eff && f ~= f0)
        % efficiency condition holds
        nEff = 1;
        ier = 0;
        break;
      end
      
      % check step size restrictions
      if (alp1 == stpmin && alp1 ~= 0.)
        % stpmin too large
        obj.print(3,Inf,'WARNING: end of line search forced by stpmin');
        nEff = 2;
        ier = 0;
        break;
      end     
      
      if (alp2 == stpmax)
        % stpmax too small
        obj.print(3,Inf,'WARNING: end of line search forced by stpmax');
        nEff = 2;
        ier = 0;
        break;
      end     
    end
    
    %********************************************
    %* compute new trial stepsize (goto150)     *
    %********************************************
    
    bSafeguard = 1;
    
    % extrapolation phase
    if (alp1 == 0.)
      % only short steps, mu>1/2, expand
      if (fbest == f0)
        % escape roundoff
        alp = alp2/sml;
        % safeguard in case fbest=f0 by chance
        if (ksh == 1)
          alp = alp2*1.1;
        end
      elseif (k == 1)
        % second trial point, safeguarded parabolic minimization
        alp = alp/max(2-2*mu,sml);
      else
        % expand by divs=cut(4*mu2,[2,4])
        divs = max(2.,min(4.,4.*mu2));
        alp = alp2*divs;
      end
      
      bSafeguard = 0;
    else
      % interpolation phase
      if (alp2 == 0.)
        % only long steps, mu<1/2, contract
        % linear interpolation to mu=0.5
        alpd = alp1/(2-2*mu1);
      elseif (fbest == f0)
        % escape roundoff by geometric mean step
        alp = alp2*sqrt(alp1/alp2);
        % safeguard in case fbest=f0 by chance
        if (ksh == 1)
          alp = min(alp,alp2*1.1);
        end
        bSafeguard = 0;
      else
        % test interiority condition for parabolic minimizer alp0
        fact = (alp1/alp2)^(1/3);
        if (denom > 0 && (alp0 >= fact*alp2 && alp1 >= fact*alp0))
          % safeguarded parabolic minimization
          alpd = alp0;
        else
          alpd = alp2;    % is updated by safeguards
        end
      end
    end
    
    if (bSafeguard)
      % compute relative step location
      q = (alpd-alp2)/(alp1-alp2);
      
      %********************************************* 
      %* safeguards                                *
      %*********************************************
      
      if (alp1 == 0)
        error('programming error in extrapolation step of line search (alp1==0)');
      elseif (alp2 == 0)
        % only long steps mu<1/2
        if (q >= sml*sml && q <= sml)
          % contract by factor q <= sml
          alp = q*alp1;
        else
          % contract by sml
          alp = sml*alp1;
        end
      else
        % force interiority condition for interpolation step
        alpd = alp2+q*(alp1-alp2);
        fact = (alp1/alp2)^(1/3);
        if (0)      %/*_isnan(f1)*/
          % f1=NaN, take a short interior step
          alpd = fact*alp2;
        else
          % f1 is a valid number or infinite
          if (alpd<fact*alp2)
            % cut interpolation step
            alpd = fact*alp2;
          end
        end
        
        alp = min(alpd,alp1/fact);
        if (alp ~= alpd || alp >= alp1 || alp<=alp2)
          % step not interior enough, use geometric mean step
          alp = alp1*sqrt(alp2/alp1);
        end
      end
    end     % of if(bSafeguard)
    
    %*********************************************
    %* check bounds and assign task  (goto2000)  *
    %*********************************************
    
    % check bounds on step size
    if (alp < stpmin)
      alp = stpmin;
    elseif (alp > stpmax)
      alp=stpmax;
    end
    
    if (alp > alp2 && (alp < alp1 || alp1 == 0.))
      % good trial step; request next function value
      % task='VALUE'
      ;
    else
      % interval too small for splitting
      if (alp2*mu2*max(omega1,omega2) >= eff && f ~= f0)
        % efficiency condition holds
        obj.print(3,Inf,'WARNING: end of line search forced by machine accuracy');
        nEff = 2;
        ier = 0;
        break;
      else
        obj.print(3,Inf,'WARNING: end of line search forced by machine accuracy');
        nEff = 0;
        ier = 1;
        break;
      end
    end
    
    if (alp2 == 0) 
      fact = alp1/alp;
    else
      fact = alp/alp2;
    end
    
    %******************************************************
    
  end    % for iterations
  
  
  %*****************************************************
  
  if (ier == 0)
    if (f < fx)   %%% by mela byt merit??
      % x=x_it;   % already done JF041111
      bLinesearch = 0;
      % fx = fnc.obj(x);
      fx=f;
      obj.eval_aldx();
      %gradx = fnc.obj_grad(x);
      nFlag=0;
    else
      if (ksh>0 && alpm > 0.) %fbest < HUGE_VAL && alpm > 0.)
        % We have descent direction, but 
        % function values cannot be improved due to 
        % numerical inexactness
        alp = alpm;
	    %x_it=x+alp*dir;
	    %x=x_it;
	    obj.xall=xall0+alp*dir;
        %if (Uequal)
	%  UEQ = u_it + alp*u_dir;
        %end
        
        bLinesearch = 0;
        
        %fx = fnc.obj(x);
        %gradx = fnc.obj_grad(x);
        obj.eval_alx();
        obj.eval_aldx();
        fx = obj.ALx;
        %gradx = fnc.obj_grad(x);
	nFlag=0;
      else
        alp = 0;
      end
    end
  else
    alp = 0;
  end
  
  lsiter=lsiter+iterations;
  obj.lsiter_last=obj.lsiter_last + lsiter;
  
  obj.print(3,Inf,'LS (pen): %.4e, %d steps, Step width: %f', pi0, iterations, alp);
  
  %return alp;
  rRelStep=alp;
  return;
  

