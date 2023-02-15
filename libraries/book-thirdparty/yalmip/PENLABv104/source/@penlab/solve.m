% main solver loop ("pennon/pbm" iterations)
function [ifail] = solve(obj)

  ifail = 0;

  % right phase?
  if (obj.phase<1)
    error('The problem needs to be initialized first.');
  else
    obj.phase=2;  % solving
  end

  PENALTY_UPDT = obj.allopts.penalty_update;
  PENALTY_UPDT_BAR = obj.allopts.penalty_update_bar;
  MAX_PBMITER = obj.allopts.max_outer_iter;
  PBMALPHA = obj.allopts.outer_stop_limit;
  KKTALPHA = obj.allopts.kkt_stop_limit;
  MU = obj.allopts.mlt_update;
  UINIT = obj.allopts.uinit;
  UINIT_BOX = obj.allopts.uinit_box;
  UINIT_EQ = obj.allopts.uinit_eq;
  UMIN = obj.allopts.umin;
  PINIT = obj.allopts.pinit;
  PINIT_BAR = obj.allopts.pinit_bar;
  USEBARRIER = obj.allopts.usebarrier;
  %visual = obj.allopts.visualisation;
  GAP_NOPROGRESS = 1e20;
  OBJ_UNBOUNDED = -1e20;
  MIN_MAX_FAILED = 3;

  % clear stats
  obj.clearstats();
  
  nFlag=length(penlab.solvermsg);  % last message: not finished yet
  starttime=cputime;

  %x=x0;  % xall? it gets projected automatically
  nGapNoProgress=0;
  nGrowingGap=0;
  nMinFailed=0;

  % do we have any constraints? (because of printing)
  bConstrIneq=obj.Nxbox+obj.Nineq+obj.NYbox+obj.NA>0;
  bConstrEq=obj.Neq>0;

  % print out basic information for better orientation in log files
  obj.print(3,Inf,'Problem name: %s',obj.probname);
  if (~isempty(obj.comment))
    obj.print(3,Inf,'Description:  %s',obj.comment);
  end
  obj.print(3,Inf,'Start time:   %s',datestr(now,0));
  obj.print_opts(3,Inf);
  obj.print(3,Inf,' ');

  % Initialize penalty parameters and Lagrangian multipliers based on
  % option settings if there are not valid values yet.
  obj.init(false);

  % automatic modification of the starting point
  % reconstruct from Nxbox constraints lbx and ubx. This is a bit silly
  % because in penlab() we have them, however, the point modification
  % should be done before each solve() (if necessary/required) thus here.
  lbx = -Inf(obj.Nx+obj.NYnnz,1);
  ubx = Inf(obj.Nx+obj.NYnnz,1);
  ind = find(obj.xboxmlt<0);
  lbx(ind) = obj.xboxshift(ind);
  ind = find(obj.xboxmlt>0);
  ubx(ind) = -obj.xboxshift(ind);
  if (obj.allopts.xinit_mod)
    % push x within the box bounds even if not treated by barrier
    % so that they are at least relative 'frac'tion from the boundary
    frac = 0.01;
    dist = frac*min(ubx - lbx,1);
    obj.xall=max(lbx+dist,obj.xall);
    obj.xall=min(ubx-dist,obj.xall);
  elseif (~isempty(obj.xboxindbar))
    % modify only x in barrier not satisfying the constraints
    % or very close to the boundary to set up a valid point
    xboxx = obj.xboxshift + obj.xboxmlt .* obj.xall(obj.xboxmap);
    ind=find(xboxx(obj.xboxindbar)>=-1e-7);
    if (~isempty(ind))
      % from box constraint numbers find variable numbers x involved
      xind=unique(obj.xboxmap(obj.xboxindbar(ind)));
      frac = 1e-5;
      dist = frac*min(ubx(xind) - lbx(xind),1);
      obj.xall(xind)=max(lbx(xind)+dist,obj.xall(xind));
      obj.xall(xind)=min(ubx(xind)-dist,obj.xall(xind));
    end
  end

  % problem information output
  nNLNineq=sum(obj.ineqmap<=obj.NgNLN);
  nNLNeq=sum(obj.eqmap<=obj.NgNLN);
  nNLNAineq=sum(obj.Amap<=obj.NANLN);
  obj.print(2,Inf,'*******************************************************************************');
  obj.print(2,Inf,penlab.solvername);
  obj.print(2,Inf,'*******************************************************************************');
  obj.print(2,Inf,'Number of variables                      %7d',obj.Nx);
  obj.print(2,Inf,'Number of matrix variables               %7d',obj.NY);
  obj.print(2,Inf,'   - degrees of freedom (var. elements)  %7d',obj.NYnnz);
  obj.print(2,Inf,'(Function) constraints');
  obj.print(2,Inf,'   - box inequalities                    %7d',obj.Nxbox);
  obj.print(2,Inf,'   - linear inequalities                 %7d',obj.Nineq-nNLNineq);
  obj.print(2,Inf,'   - nonlinear inequalities              %7d',nNLNineq);
  obj.print(2,Inf,'   - linear equalities                   %7d',obj.Neq-nNLNeq);
  obj.print(2,Inf,'   - nonlinear equalities                %7d',nNLNeq);
  obj.print(2,Inf,'Matrix constraints');
  obj.print(2,Inf,'   - box inequalities                    %7d',obj.NYbox);
  obj.print(2,Inf,'   - linear inequalities                 %7d',obj.NA-nNLNAineq);
  obj.print(2,Inf,'   - nonlinear inequalities              %7d',nNLNAineq);
  obj.print(2,Inf,' ');
  if (obj.Nxbox>0)
    obj.print(2,Inf,'Min./Max. box-mult.:   %9f / %9f',min(obj.uxbox),max(obj.uxbox));
  end
  if (obj.Nineq>0)
    obj.print(2,Inf,'Min./Max. ineq-mult.:  %9f / %9f',min(obj.uineq),max(obj.uineq));
  end
  if (obj.Neq>0)
    obj.print(2,Inf,'Min./Max. equal-mult.: %9f / %9f',min(obj.ueq),max(obj.ueq));
  end

  % evaluation of AL will update all objx, xboxx, ineqx, eqx ...
  % and we nned its derivative anyway...
  obj.eval_alx();
  obj.eval_aldx();
  % just in case I use old notation... TODO remove it later and use obj.*
  % directly
  f=obj.objx;
  constrx=[obj.xboxx;obj.ineqx];
  eqx=obj.eqx;
  lagrx=obj.ALx;
  gradx=obj.ALdx;
  gradeqx=obj.eqdx;
  uineq_updt = [];
  uxbox_updt = [];

  obj.rNormG=norm(obj.ALdx);
  rFeasM=obj.feas_ay();
  obj.rFeas=max([abs(obj.eqx);max(0,obj.xboxx);max(0,obj.ineqx);rFeasM;0]);
  rGap=abs(obj.objx-obj.ALx);
  if (bConstrIneq)
    rPmin=min([obj.pxbox;obj.pineq;obj.PYbox;obj.PA]);
  else
    rPmin=0;
  end
  if (bConstrIneq || bConstrEq)
    %rCompl=max(abs(u.*constrx));
    % TODO ... what if is somewhere barrier?? (and using uxbox??)
    % TODO matrix complementarity??
    obj.rCompl=max([0;abs(obj.uxbox.*obj.xboxx);abs(obj.uineq.*obj.ineqx);abs(obj.eqx.*obj.ueq)]);
      %%% ^^^ all right? but ueq is 0 (if by default) anyway... but might be by DCF
  else
    obj.rCompl=0;
  end
  % detailed starting information
  obj.print(3,Inf,'******************** Start *********************');
  obj.print(3,Inf,'Objective            %27.16E',obj.objx);
  obj.print(3,Inf,'Augmented Lagrangian %27.16E',obj.ALx);
  obj.print(3,Inf,'|f(x) - Lagr(x)|     %27.16E',rGap);
  obj.print(3,Inf,'Grad augm. lagr.     %27.16E',obj.rNormG);
  obj.print(3,Inf,'Feasibility (max)    %27.16E',obj.rFeas);
  obj.print(3,Inf,'Feasibility eqx      %27.16E',max(abs(obj.eqx)));
  obj.print(3,Inf,'Feasibility ineq     %27.16E',max(max(0,obj.ineqx)));
  obj.print(3,Inf,'Feasibility box      %27.16E',max(max(0,obj.xboxx)));
  obj.print(3,Inf,'Feasibility m.ineq   %27.16E',rFeasM);
  obj.print(3,Inf,'Complementarity      %27.16E',obj.rCompl);
  obj.print(3,Inf,'Minimal penalty      %27.16E',rPmin);
  obj.print(3,Inf,'************************************************');
  obj.print(3,Inf,' ');

  obj.print(2,3,'*******************************************************************************');
  obj.print(2,3,'* it |     obj      | (U,G(x)) |  ||dF||  |   feas   |   pmin   |  Nwt | InIt |');
  obj.print(2,3,'*******************************************************************************');
  if (bConstrIneq || bConstrEq)
    obj.print(2,3,'| %3d|%13.5e |%9.1e |%9.1e |%9.1e |%9.1e | %4d | %4d |',0,obj.objx,obj.rCompl,obj.rNormG,obj.rFeas,rPmin,obj.miter_last,obj.initer_last);
  else
    obj.print(2,3,'| %3d|%13.5e |          |%9.1e |          |          | %4d | %4d |',0,obj.objx,obj.rNormG,obj.miter_last,obj.initer_last);
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  nFlag=7;
  for pbmiter=1:MAX_PBMITER

    f_old = obj.objx;
    rGap_old = rGap;

    obj.print(3,Inf,'************* Start of outer step %3i **********',pbmiter);

    % TODO tracing information?

    % unconstr minimization step
    if (obj.Neq==0)
        %fnc.obj=@(xtmp) auglagr(xtmp,u,p,'',ps);
        %fnc.obj_grad=@(xtmp) auglagr_D(xtmp,u,p,'',ps);
        %fnc.obj_hess=@(xtmp) auglagr_D2(xtmp,u,p,'',ps);

        [nFlagMin,rResults]=obj.unconstr_min();
        lagrx=rResults(1);
        obj.rNormG=rResults(2);
        % or without rResults??
        %lagrx=obj.ALx;
        %obj.rNormG=norm(obj.ALdx);
    else
        %fnc.obj=@(xtmp,ueqtmp) auglagr(xtmp,u,p,ueqtmp,ps);
        %fnc.obj_grad=@(xtmp,ueqtmp) auglagr_D(xtmp,u,p,ueqtmp,ps);
        %fnc.obj_hess=@(xtmp,ueqtmp) auglagr_D2(xtmp,u,p,ueqtmp,ps);

        [nFlagMin,rResults]=obj.eqconstr_min();
        lagrx=rResults(1);
        obj.rNormG=rResults(2);
        % or directly?
        %lagrx=obj.ALdx
        %obj.rNormG=norm(obj.ALddx);
    end

    % these should be already evaluated by linesearch ... just to be sure
    % evaluation of AL will update all objx, xboxx, ineqx, eqx ...
    % and we nned its derivative anyway...
    obj.eval_alx();
    obj.eval_aldx();

    % trial (full) update of Lagrangian multipliers
    % (compute update of the multipliers as if it wasn't restricted by MU)
    if (obj.Nxbox+obj.Nineq~=0)
      %if (USEBARRIER)   % update only multipliers for 'non-barriered' constraints
      %  indbar=[1:ps.N_BOUNDS];
      %  ind=[ps.N_BOUNDS+1:ps.N_CONSTR-ps.N_EQUAL];
      %else
      %  indbar=[];
      %  ind=[1:ps.N_CONSTR-ps.N_EQUAL];
      %end	

      %u_updt(barind) = p(barind).*phibar_D(constrx(barind));
      %u_updt(phiind) = phi2_D(constrx(phiind)./p(phiind));
      uineq_updt = obj.phi2_D(obj.ineqx./obj.pineq);
      uxbox_updt = obj.phi2_D(obj.xboxx./obj.pxbox);
    end

    % print results
    % just in case I use old notation... TODO remove it later and use obj.*
    % directly
    f=obj.objx;
    constrx=[obj.xboxx;obj.ineqx];
    eqx=obj.eqx;
    lagrx=obj.ALx;
    gradx=obj.ALdx;
    gradeqx=obj.eqdx;

    obj.rNormG=norm(obj.ALdx);
    rFeasM=obj.feas_ay();
    obj.rFeas=max([abs(obj.eqx);max(0,obj.xboxx);max(0,obj.ineqx);rFeasM;0]);
    rGap=abs(obj.objx-obj.ALx);
    if (bConstrIneq)
      % TODO this hasn't been change, has it?
      rPmin=min([obj.pxbox;obj.pineq;obj.PYbox;obj.PA]);
    end
    if (bConstrIneq || bConstrEq)
      % TODO ... what if is somewhere barrier?? (and using uxbox??)
      % TODO matrix complementarity??
      obj.rCompl=max([0;abs(obj.uxbox.*uxbox_updt.*obj.xboxx);abs(obj.uineq.*uineq_updt.*obj.ineqx);abs(obj.eqx.*obj.ueq)]);
    end

    if (nFlagMin>0)  % solver failed
      obj.print(3,Inf,'*****!!!!!!! Result of outer step %3i !!!!!*****',pbmiter);
    else
      obj.print(3,Inf,'************ Result of outer step %3i **********',pbmiter);
    end
    obj.print(3,Inf,'Objective            %27.16E',obj.objx);
    obj.print(3,Inf,'Augmented Lagrangian %27.16E',obj.ALx);
    obj.print(3,Inf,'|f(x) - f(x_old)|    %27.16E',abs(f-f_old));
    obj.print(3,Inf,'|f(x) - Lagr(x)|     %27.16E',rGap);
    obj.print(3,Inf,'Grad augm. lagr.     %27.16E',obj.rNormG);
    obj.print(3,Inf,'Feasibility (max)    %27.16E',obj.rFeas);
    obj.print(3,Inf,'Feasibility eqx      %27.16E',max(abs(obj.eqx)));
    obj.print(3,Inf,'Feasibility ineq     %27.16E',max(max(0,obj.ineqx)));
    obj.print(3,Inf,'Feasibility box      %27.16E',max(max(0,obj.xboxx)));
    obj.print(3,Inf,'Feasibility m.ineq   %27.16E',rFeasM);
    obj.print(3,Inf,'Complementarity      %27.16E',obj.rCompl);
    obj.print(3,Inf,'Minimal penalty      %27.16E',rPmin);
    obj.print(3,Inf,'Newton steps                               %5d',obj.miter_last);
    obj.print(3,Inf,'Inner steps                                %5d',obj.initer_last);
    obj.print(3,Inf,'Linesearch steps                           %5d',obj.lsiter_last);
    obj.print(3,Inf,'Time of the minimization step   %14g s',obj.stats_time_miter_last);
    obj.print(3,Inf,'  - factorizations in the step  %14g s',obj.stats_time_fact_last);
    if (nFlagMin>0)   % solver failed
      obj.print(3,Inf,'*****!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*****');
      warn_sign='!';
    else
      obj.print(3,Inf,'************************************************');
      warn_sign=' ';
    end 
    obj.print(3,Inf,' ');

    if (bConstrIneq || bConstrEq)
      obj.print(2,3,'| %3d|%13.5e |%9.1e |%9.1e |%9.1e |%9.1e | %4d | %4d%s|',pbmiter,obj.objx,obj.rCompl,obj.rNormG,obj.rFeas,rPmin,obj.miter_last,obj.initer_last,warn_sign);
    else
      obj.print(2,3,'| %3d|%13.5e |          |%9.1e |          |          | %4d | %4d%s|',pbmiter,obj.objx,obj.rNormG,obj.miter_last,obj.initer_last,warn_sign);
    end

    % visualisation of VTS problems??
    %if (visual)
    %  vis_vts3('',x);
    %end

    % stoping criterion
    if (nFlagMin>0)
      nMinFailed=nMinFailed+1;
      obj.print(3,Inf,'Warning: Unconstr/eqconstr. min failed (flag %i), remaing attepmts: %i',nFlagMin,MIN_MAX_FAILED-nMinFailed);
      % perhaps don't update penalty so strongly?
    else
      nMinFailed=0;
    end
    if (nMinFailed >= MIN_MAX_FAILED)
      nFlag=5;
      break;
    end

    %if (obj.rNormG<PBMALPHA)
    %if (obj.rNormG<PBMALPHA && (isempty(obj.rFeas) || obj.rFeas<PBMALPHA))
    if (stop_crit(obj.objx, f_old, PBMALPHA) && stop_crit(obj.objx, obj.ALx, 10*PBMALPHA) && (obj.rFeas<1e-4*abs(obj.ALx) || obj.rFeas<1e-3))
      if ( (obj.rNormG<KKTALPHA && obj.rCompl<KKTALPHA && obj.rFeas<KKTALPHA) || (obj.rNormG<10*KKTALPHA && obj.rCompl<0.01*KKTALPHA && obj.rFeas<0.01*KKTALPHA) )
        nFlag=1;
        break;
      end
    end
    if (rGap > GAP_NOPROGRESS)
      nGapNoProgress=nGapNoProgress+1;
    else
      nGapNoProgress=0;
    end
    if (rGap > rGap_old)
      nGrowingGap=nGrowingGap+1;
    else
      nGrowingGap=0;
    end
    if (nGapNoProgress>2 || nGrowingGap>40)
      nFlag=6;  % no progress, stop it
      break;
    end
    if (obj.ALx<OBJ_UNBOUNDED)
      nFlag=4;  % objective seems to be unbounded, stop it
      break;
    end

    % (restricted) multipliers update
    obj.uxbox = lmlt_update(obj.uxbox,uxbox_updt,obj.xboxindphi,MU,UMIN);
    obj.uineq = lmlt_update(obj.uineq,uineq_updt,obj.ineqindphi,MU,UMIN);
    % (restricted) matrix multipliers update
    obj.lmltm_update();
    % penalty update
    if (obj.Nxbox>0)
      obj.pxbox(obj.xboxindbar) = obj.pxbox(obj.xboxindbar).*PENALTY_UPDT_BAR;
      obj.pxbox(obj.xboxindphi) = obj.pxbox(obj.xboxindphi).*PENALTY_UPDT;
    end
    if (obj.Nineq>0)
      %obj.pineq(obj.ineqindbar) = obj.pineq(obj.ineqindbar).*PENALTY_UPDT_BAR;
      obj.pineq(obj.ineqindphi) = obj.pineq(obj.ineqindphi).*PENALTY_UPDT;
    end
    % matrix penalty update
    obj.mpen_update();
    
    % force recompute ALx, ALdx, ALddx (point hasn't change so
    % still valid ticker but because penalties and Lagrangian
    % changed, AL*x is actually not valid)
    % this is a bit dirty, ideally P(:) and U(:) should have
    % a counter as well and ALx would be max(xTck, PTck, UTck)
    obj.ALxtck=0;
    obj.ALdxtck=0;
    obj.ALddxtck=0;

  end    % of pbmiter cycle
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % full multipliers update if not terminated by maxit
  %[f,constrx,eqx] = ps.func_val(x); % not necessary, x unchanged
  %if (nFlag~=4 && ps.N_CONSTR-ps.N_EQUAL~=0)
  %  u = u.*u_updt;
  %end
  % (full) multipliers update
  % TODO update it always? even if error occured
  obj.uxbox(obj.xboxindphi) = obj.uxbox(obj.xboxindphi).*uxbox_updt(obj.xboxindphi);
  obj.uineq(obj.ineqindphi) = obj.uineq(obj.ineqindphi).*uineq_updt(obj.ineqindphi);

  % print results
  obj.rNormG=norm(obj.ALdx);
  obj.rFeas=max([abs(obj.eqx);max(0,obj.xboxx);max(0,obj.ineqx);0]);
  rGap=abs(obj.objx-obj.ALx);
  if (obj.Nxbox + obj.Nineq>0)
    % TODO this hasn't been change, has it?
    rPmin=min([obj.pxbox;obj.pineq]);
  end
  if (obj.Nxbox + obj.Nineq + obj.Neq>0)
    % TODO ... what if is somewhere barrier?? (and using uxbox??)
    obj.rCompl=max([abs(obj.uxbox.*obj.xboxx);abs(obj.uineq.*obj.ineqx);abs(obj.eqx.*obj.ueq)]);
  end
  %  ??? not necessary to recompute?
  % Note: if finished with maxit -> lagrangian multipliers weren't fully updated --> printed slackness may be absolutely wrong; better to leave the earlier computed one
    
  if (nFlag==1)
    if (obj.rNormG>1.0e-4*abs(obj.ALx) && obj.rNormG>1.0e-3)
      nFlag=2;
    end
    if(obj.rFeas>1.0e-4*abs(obj.ALx) && obj.rFeas>1.0e-3)
      nFlag=3;
    end
  end
  obj.stats_time_total = cputime - starttime;

  obj.print(2,Inf,'*******************************************************************************');
  if (nFlag>=1 && nFlag<length(penlab.solvermsg)-1)
    obj.print(2,Inf,'PenLab %s',penlab.solvermsg{nFlag});
  else
    obj.print(2,Inf,'PenLab: Unknow error (%i)',nFlag);
  end
  obj.print(2,Inf,'*******************************************************************************');
  obj.print(2,Inf,'Objective            %27.16E',obj.objx);
  obj.print(3,Inf,'Augmented Lagrangian %27.16E',obj.ALx);
  obj.print(2,Inf,'Relative precision   %27.16E',abs(obj.ALx-obj.objx)/max(1,obj.objx));
  obj.print(2,Inf,'Compl. Slackness     %27.16E',obj.rCompl);
  obj.print(2,Inf,'Grad augm. lagr.     %27.16E',obj.rNormG);
  obj.print(2,Inf,'Feasibility (max)    %27.16E',obj.rFeas);
  obj.print(3,Inf,'Feasibility eqx      %27.16E',max(abs(obj.eqx)));
  obj.print(3,Inf,'Feasibility ineq     %27.16E',max(max(0,obj.ineqx)));
  obj.print(3,Inf,'Feasibility box      %27.16E',max(max(0,obj.xboxx)));
  obj.print(3,Inf,'Minimal penalty      %27.16E',rPmin);
  obj.print(2,Inf,'Newton steps                               %5d',obj.miter);
  obj.print(2,Inf,'Inner steps                                %5d',obj.initer);
  obj.print(2,Inf,'Linesearch steps                           %5d',obj.lsiter);
  obj.print(2,Inf,'Number of evaluations of');
  obj.print(2,Inf,'   - function values                       %5d',obj.stats_ncall_alx);
  obj.print(2,Inf,'   - gradient values                       %5d',obj.stats_ncall_aldx);
  obj.print(2,Inf,'   - hessian values                        %5d',obj.stats_ncall_alddx);
  obj.print(2,Inf,'Time statistics');
  obj.print(2,Inf,'   - total process time         %14g s',obj.stats_time_total);
  obj.print(2,Inf,'   - all minimization steps     %14g s',obj.stats_time_miters);
  obj.print(2,Inf,'   - all factorizations         %14g s',obj.stats_time_fact);
  obj.print(2,Inf,'   - function values evaluation %14g s',obj.stats_time_alx);
  obj.print(2,Inf,'   - gradient values evaluation %14g s',obj.stats_time_aldx);
  obj.print(2,Inf,'   - hessian values evaluation  %14g s',obj.stats_time_alddx);
  obj.print(2,Inf,'*******************************************************************************');
  obj.print(2,Inf,' ');

  obj.phase=3;   % solver finished

  ifail = nFlag;
end

%%%%%%%%%%%%%%% Helper routines %%%%%%%%%%%%%%%%%

% basic stopping criterion
function [b]=stop_crit(f1,f2,eps)

  b = abs(f1 - f2) < eps*(1+.5*(abs(f2)+abs(f1)));
  return
end

% update of lagrangian multipliers with 'mu' limitation and minimum
% value check
function [u] = lmlt_update(u,u_updt,phiind,mu,umin)
  if (~isempty(phiind))
    utmp = u(phiind).*u_updt(phiind);
    utmp = min((1/mu)*u(phiind), utmp);
    u(phiind) = max(mu*u(phiind), utmp);
    % check for multiplier minima (UMIN)
    xchange=abs(u)<umin & u~=0;
    u(xchange)=sign(u(xchange))*umin;
  end
end

