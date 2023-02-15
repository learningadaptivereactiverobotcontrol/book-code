% PenLab (Pennon Laboratory for Matlab, previously PennonM)
% a NLP-SDP optimization solver
classdef penlab < handle
  
  properties   % fully public
    % Problem (object) name with optional comments
    probname = 'No problem loaded'; 
    comment = '';

    % user data structure to be passed to callbacks
    userdata = [];

    % (accepted) user's option settings
    opts = struct();

    % starting point given by user
    xinit = [];
    Yinit = [];

  end

  properties (SetAccess = private)
    % phase of the object, 0 ~ EMPTY, 1 ~ INIT, 2 ~ SOLVING, 3 ~ FINISHED
    phase = 0;

    % number of variables (in a vector)
    Nx = 0;
    % number of matrix variables
    NY = 0;
    % number of variables in all matrix variables together
    NYnnz = 0;
    % how are variables mapped into matricies Y, just to inform user
    Ymap = [];

    % number of nonlinear constraint functions
    NgNLN = 0;
    % number of linear constraint functions
    NgLIN = 0;
    % number of nonlinear matrix constraint functions
    NANLN = 0;
    % number of linear matrix constraint functions
    NALIN = 0;

    % current point (or result), only updated via get.x(), get.Y() as
    % a projection of xall
    x = [];
    Y = [];

    %% Statistics
    % no of objfun calls ...
    % no of outer iterations (completed)
    % no of inner iterations (completed)
    stats_ncall_alx = 0;
    stats_time_alx = 0;
    stats_ncall_aldx = 0;
    stats_time_aldx = 0;
    stats_ncall_alddx = 0;
    stats_time_alddx = 0;
    stats_time_fact_last = 0;
    stats_time_fact = 0;
    stats_time_miter_last = 0;
    stats_time_miters = 0;
    stats_time_total = 0;
    miter = 0;
    miter_last = 0;
    initer = 0;
    initer_last = 0;
    lsiter = 0;
    lsiter_last = 0;
    % add major iteration counter, inner iteration counter, LS steps
    % time factor, total

    % status/meassures while solving and when solver finished
    solveflag = [];
    rFeas = [];
    rCompl = [];
    rNormG = [];

    % or set it up as 'empty routine'?
    % objfun = @(x,Y,userdata) deal([],userdata);
    objfun = [];
    objgrad = [];
    objhess = [];
    confun = [];
    congrad = [];
    conhess = [];
    lagrhess = [];
    mconfun = [];
    mcongrad = [];
    mconhess = [];
    mconlagrhess = [];

    % all option settings (default + user's)
    % allopts is linked with opts via set.opts() and shouldn't be changed
    % directly
    allopts = penlab.defopts(1);

    %%%%%%%%%%%%%%%%% MAKE ME PRIVATE LATER %%%%%%%%%%%%%%%%%%%%%%
    % real number of constraints

    % number of box constraints on X
    Nxbox = 0;
    % mapping: Nxbox (internal numebring) -> Nx bounds
    xboxmap = [];
    xboxmlt = [];
    xboxshift = [];
    % number of matrix box constraints (no of inequal on Y matrix variables)
    NYbox = 0;
    % vectorized variable (xall(Nx+1:Nx+NYnnz) into Y map
    vec2Ymap = [];
    % mapping: no "box" matrix inequality constraint -> k  of appropriate Y{k}
    Yboxmap = [];
    Yboxmlt = [];
    Yboxshift = [];
    % box constraints on the elements of the matrix variables <== among xbox*
    %NYxbox = 0;
    %Yxboxmap = [];
    %Yxboxmlt = [];
    %Yxboxshift = [];
    % equality on the elements...? <== at the moment 2 inequalities
    %NYxboxeq = 0;
    %Yxboxmapeq = [];
    %Yxboxshifteq = [];

    % function inequalities, nonlin & lin merged (but in this order)
    %NineqNLN = 0;
    %NineqLIN = 0;
    Nineq = 0;
    ineqmap = [];
    ineqmlt = [];
    ineqshift = [];
    % function equalities, nonlin & lin
    Neq = 0;
    eqmap = [];
    eqshift = [];
    % matrix inequalitites, nonlin & lin merged (kept in order)
    NA = 0;
    Amap = [];
    Amlt = [];
    Ashift = [];
    % list of dependent variables on A(k) k=1..NANLN+NALIN, i.e., the original
    % numbering of matrix constraints
    Adep=[];

    % type of the penalty used for function and matrix inequalitites
    xboxtype = [];
    ineqtype = [];
    Yboxtype = [];
    Atype = [];

    % type changed into list of indicies (use setpentype())
    xboxindbar = [];
    xboxindphi = [];
    %ineqindbar = [];
    ineqindphi = [];
    Yboxindbar = [];
    Yboxindphi = [];
    %Aindbar = [];
    Aindphi = [];


  end

  properties %(SetAccess = private, GetAccess = private)

    % time ticker (with every change of xall adds 1)
    ticker = 1;
    % (users) x and Y copy (projection) of xall
    xYtck = 0;


    % all variables, current internal point
    xall = [];

    % Lagrangian multipliers for box inequalities, function ineq. & eq. and new
    % candidates
    uxbox = [];
    uxboxnew = [];
    uineq = [];
    uineqnew = [];
    ueq = [];
    % Lagrangian multipliers for matrix inequalities: box constr. of matrix 
    % variables and matrix constraints inequalities
    UYbox = [];
    UYboxnew = [];
    UA = [];
    UAnew = [];

    % Penalty parameters for all... (except equalities obviously)
    pxbox = [];
    pineq = [];
    PYbox = [];
    PA = [];

    %%% Various values of transformed user data
    % TODO add ticker!!!
    objx = 0;
    xboxx = [];
    ineqx = [];
    eqx = [];
    % ticker and Jacobian of equalities
    eqdxtck = 0;
    eqdx = [];

    % ticker of the augmented lagrangian and its value at 'xall' point
    % and value of all function equalitites
    ALxtck = 0;
    ALx = 0;

    % ticker and the value of the 1st derivative of ALx
    ALdxtck = 0;
    ALdx = [];

    % ticker and the value of the Hessian matrix of the Augmented Lagrangian
    ALddxtck = 0;
    ALddx = [];

    % stream for log file output
    fid = -1;

    % solve_chol & solvekkt_ldl specific
    chol_lmlast = 0;     % the last nonzero diagonal perturbation
    chol_factperm = [];  % nontrivial permutation used in the last attempt
    % linesearch specific
    ls_small_cnt = 0;
    ls_rnu = [];
  end

  properties (Constant)
    % solver name & version
    solvername = 'PenLab 1.04 (20140125)';

    % final solver message based on solverflag
    solvermsg = {'converged: optimal solution', ...
       'converged: suboptimal solution', ...
       'finished: solution primal infeasible', ...
       'finished: objective function seems to be unbounded', ...
       'didn''t converge: unconstrained minimization failed', ...
       'didn''t converge: no progress, iteration process terminated', ...
       'didn''t converge: outer iterations reached', ...
       'didn''t converge: unknown error', ...
       'hasn''t finished yet'};

    % default options
    % struct array: name, {default_value, type, restriction}
    % where  name is the name of the option used in allopts or opts
    %        default_value   (==> defopts(1) ~ default allopts structure)
    %        restriction ... depending on the type, restriction to the accepted values
    %        type: 'O' other, no restriction, no checking (e.g. usr_prn)
    %              'S' string, no restriction
    %              'I', 'R' integer or real within range restriction(1)~restriction(2)
    %              'M' multiple choice - one of the elements in array restriction
    defopts = struct(...
      'outlev', {2, 'I', [0, Inf]}, ...
      'outlev_file', {5, 'I', [0, Inf]}, ...
      'out_filename', {'penm_log.txt', 'S', []}, ...
      'user_prn', {[], 'O', []}, ...
      'maxotiter', {100, 'I', [0, Inf]}, ...
      'maxiniter', {100, 'I', [0, Inf]}, ...
      ... % from pennon.m
      'penalty_update', {0.5, 'R', [0, 1]}, ...       % PENALTY_UPDT
      'penalty_update_bar', {0.3, 'R', [0, 1]}, ...   % PENALTY_UPDT_BAR
      'mpenalty_update', {0.5, 'R', [0, 1]}, ...       % 
      'mpenalty_min', {1e-6, 'R', [0, 1]}, ...       % 
      'mpenalty_border', {1e-6, 'R', [0, 1]}, ...       % 
      'max_outer_iter', {100, 'I', [0, Inf]}, ...       % MAX_PBMITER
      'outer_stop_limit', {1e-6, 'R', [1e-20, 1]}, ...    % PBMALPHA
      'kkt_stop_limit', {1e-4, 'R', [1e-20, 1]}, ...      % KKTALPHA
      'mlt_update', {0.3, 'R', [0, 1]}, ...            % MU
      'mmlt_update', {0.1, 'R', [0, 1]}, ...            % MU2
      'uinit', {1, 'R', [-Inf, Inf]}, ...                  % UINIT
      'uinit_box', {1, 'R', [-Inf, Inf]}, ...              % UINIT_BOX
      'uinit_eq', {0, 'R', [-Inf, Inf]}, ...               % UINIT_EQ
      'umin', {1e-10, 'R', [0, 1]}, ...               % UMIN
      'pinit', {1, 'R', [0, 1]}, ...                  % PINIT
      'pinit_bar', {1, 'R', [0, 1]}, ...              % PINIT_BAR
      'usebarrier', {0, 'M', [0, 1]}, ...             % USEBARRIER
      'xinit_mod', {0, 'M', [0, 1]}, ...             % modification of xinit
      ... % from unconstr_min.m
      'max_inner_iter', {100, 'I', [0, Inf]}, ...     % MAX_MITER
      'inner_stop_limit', {1e-2, 'R', [0, 1]}, ...    % ALPHA
      'unc_dir_stop_limit', {1e-2, 'R', [0, 1]}, ...  % TOL_DIR
      'unc_solver', {0, 'M', [0, 1, 2, 3, 4, 5]}, ... % solver
      'unc_linesearch', {3, 'M', [0, 1, 2, 3]}, ...   % linesearch
      ... % from eqconstr_min.m
      'eq_dir_stop_limit', {1e-2, 'R', [0, 1]}, ...   % TOL_DIR
      'eq_solver', {0, 'M', [0, 1]}, ...              % solver
      'eq_linesearch', {3, 'M', [0, 1, 2, 3]}, ...    % linesearch
      'eq_solver_warn_max', {4, 'I', [0, 10]}, ...    % solver_warn_max
      'ls_short_max', {3, 'I', [0, 10]}, ...          % ls_short_max
      'min_recover_strategy', {0, 'M', [0, 1]}, ...   % recover_strategy
      'min_recover_max', {3, 'I', [0, 10]}, ...       % recover_max
      ... % from phi2.m
      'phi_R', {-0.5, 'R', [-1, 1]}, ...              % R_default
      ... % linesearchs - ls_armijo.m
      'max_ls_iter', {20, 'I', [0, Inf]}, ...   % max tries before LS fails
      'max_lseq_iter', {20, 'I', [0, Inf]}, ...   % same for LS for equality constrained problems
      'armijo_eps', {1e-2, 'R', [0, 1]}, ...  % when is armijo step satisfactory? P(alp) - P(0) <= eps*alp*P'(0)
      ... % solve_simple_chol.m, solkvekkt_ldl.m, solvekkt_lu.m
      'ldl_pivot', {1e-5, 'R', [0, 1]}, ...  % pivot tolerance for delayed pivots in LDL
      'pert_update', {2., 'R', [0, 100]}, ...  % known aka LMUPDATE, multiplier of the lambda-perturbation factor
      'pert_min', {1e-6, 'R', [0, 1]}, ...   % LMLOW, minimal (starting) perturbation
      'pert_try_max', {50, 'I', [0, Inf]}, ... % max number of attempts to successfully perturbate a matrix
      'pert_faster', {1, 'M', [0, 1]}, ...   % use the last known negative curvature vector to determine perturbation
      ... % solve_simple_chol.m
      'chol_ordering', {1, 'M', [0, 1]}, ... % use symamd for sparse matrices before Cholesky factor.?
      ... % solvers
      'luk3_diag', {1., 'R', [0, Inf]} ...  % diagonal of the (2,2)-block in Luksan 3 preconditioner
    );



  end

  methods
    Y = vec2Y(obj, vec);
    vec = Y2vec(obj, Y, Ydefault);
    [errmsg] = print(obj, minlev, maxlev, msg, varargin);
    [errmsg] = print_opts(obj, minlev, maxlev);
    []=setpentype(obj,lbxbar,ubxbar,lbYbar,ubYbar);
    [] = init(obj, forceupdate);
    [ret] = phi2(obj,t);
    [ret] = phi2_D(obj,t);
    [ret] = phi2_D2(obj,t);
    [ret] = phibar(obj,t);
    [ret] = phibar_D(obj,t);
    [ret] = phibar_D2(obj,t);
    [status] = eval_alx(obj);
    [status] = eval_aldx(obj);
    [status] = eval_alddx(obj);
    [] = clearstats(obj);
    [dir,nFlag]=solve_chol(obj,matrix,rhs);
    [x_dir,ueq_dir,nFlag,pert]=solvekkt_ldl(obj,H,A,rhs1,rhs2);
    [rRelStep, nFlag]=ls_pennon(obj, dir);
    [rRelStep, nFlag]=lseq_pen(obj, dir, udir, miter);
    [nFlag,rResults]=unconstr_min(obj);
    [nFlag,rResults]=eqconstr_min(obj);
    [] = logfile(obj,task);
    disp(obj);
    [mfeas] = feas_ay(obj);

    % constructor, you need to provide 'penm' structure
    function obj = penlab(penm)
      if (nargin > 0)
        if (isstruct(penm))

          % fill in nonexistent (optional) fields
          penm=defaultsfiller(penm);

          % open the filestream if needed
          obj.logfile(1);

          % first options to set up printing
          %obj.opts=penm.opts;

          obj.probname=penm.probname;
          obj.comment=penm.comment;

          obj.userdata=penm.userdata;


          if (penm.Nx<0)
            error('ERR: wrong Nx');
          else
            obj.Nx=penm.Nx;
            % equalities not allowed, unconstrained are OK
            [obj.xboxmap, obj.xboxmlt, obj.xboxshift]= ...
               boundschecker(penm.Nx,penm.lbx,penm.ubx,false,true,[]);
            obj.Nxbox = length(obj.xboxmap);
          end

          if (penm.NY<0)
            error('ERR: wrong NY')
          else
            obj.NY=penm.NY;
            obj.NYnnz=0;
            filterout=[];

            for k=1:penm.NY
              [mapper, obj.NYnnz]=createY1map(penm.Y{k}, obj.NYnnz);
              obj.vec2Ymap{k}=mapper;
              if (isempty(mapper.xmap))
                % ignore/exclude from matrix inequality constraints
                filterout = [filterout, k];
              end
            end

            % box equalities on elements of Y (in my notation 'Yx' stuff)
            % what to do with different bounds on the "same" elements 
            % (e.g. 5<=x_45 but 3<=x_54) ? --> only lower diag is considered
            % get vectorized lower & upper values (+/-Inf for defaults)
            lbYxvec=obj.Y2vec(penm.lbYx,-Inf);
            ubYxvec=obj.Y2vec(penm.ubYx,Inf);
            % map inequalities after 'Nx' box inequalitites, equal on elemets
            % is ok but put as 2 inequal, unconstr is allowed
            [ineqmap, ineqmlt, ineqshift]= ...
               boundschecker(obj.NYnnz,lbYxvec,ubYxvec,-1,true,[]);
            ineqmap=ineqmap+obj.Nx;
            obj.xboxmap = [obj.xboxmap, ineqmap];
            obj.xboxmlt = [obj.xboxmlt, ineqmlt];
            obj.xboxshift = [obj.xboxshift, ineqshift];
            obj.Nxbox = obj.Nxbox + length(ineqmap);

            % equalities not allowed, empty matrices filtered out
            [obj.Yboxmap, obj.Yboxmlt, obj.Yboxshift]= ...
               boundschecker(penm.NY,penm.lbY,penm.ubY,false,true,filterout);
            obj.NYbox = length(obj.Yboxmap);

            % how xall (matrix variable) part maps into Y...
            obj.Ymap = obj.vec2Y([obj.Nx+1:obj.Nx+obj.NYnnz]);
          end

          % any decision variables at all?
          if (obj.Nx+obj.NYnnz==0)
            error('ERR: no decision variables set!');
          end

          % starting point (unless user offers a better one later)
          obj.xall=zeros(obj.Nx+obj.NYnnz,1);
          if (isfield(penm,'xinit'))
            obj.xinit=penm.xinit;
          end
          if (isfield(penm,'Yinit'))
            obj.Yinit=penm.Yinit;
          end

          if (penm.NgNLN<0 || penm.NgLIN<0)
            error('Ng*<0 error');
          else
            obj.NgNLN=penm.NgNLN;
            obj.NgLIN=penm.NgLIN;

            % equalities are allowed, unconstrained not
            [obj.ineqmap, obj.ineqmlt, obj.ineqshift, obj.eqmap, obj.eqshift]= ...
               boundschecker(penm.NgNLN+penm.NgLIN,penm.lbg,penm.ubg,true,false,[]);
            obj.Nineq = length(obj.ineqmap);
            obj.Neq = length(obj.eqmap);

          end

          if (penm.NANLN<0 || penm.NALIN<0)
            error('NA*<0 error');
          else
            obj.NANLN=penm.NANLN;
            obj.NALIN=penm.NALIN;
            % neither equalities inor unconstr are allowed
            [obj.Amap, obj.Amlt, obj.Ashift]= ...
               boundschecker(penm.NANLN+penm.NALIN,penm.lbA,penm.ubA,false,false,[]);
            obj.NA = length(obj.Amap);


          end

          % Are callbacks present?
          if (~isfield(penm,'objfun') || ~isfield(penm,'objgrad'))
            % could consider it as feasible point problem but at the moment
            % it's an error
            error('objfun and/or objgrad not defined');
          end
          obj.objfun=penm.objfun;
          obj.objgrad=penm.objgrad;
          if (obj.NgNLN+obj.NgLIN>0)
            if (~isfield(penm,'confun') || ~isfield(penm,'congrad'))
              error('confun and/or congrad not defined and constraint are present');
            end
            obj.confun=penm.confun;
            obj.congrad=penm.congrad;
          end
          if (isfield(penm,'lagrhess'))
            % hessian of the Lagrangian will be used rather hess of obj & constr
            obj.lagrhess=penm.lagrhess;
          else
            if (isfield(penm,'objhess'))
              obj.objhess=penm.objhess;
            else
              error('Neither lagrhess nor objhess is defined.');
            end
            if (obj.NgNLN>0 && ~isfield(penm,'conhess'))
              error('There are nonlinear function constraints and neither lagrhess nor conhess is defined.');
            elseif (obj.NgNLN>0)
              obj.conhess=penm.conhess;
            end
          end
          if (obj.NALIN+obj.NANLN>0)
            if (~isfield(penm,'mconfun') || ~isfield(penm,'mcongrad'))
              error('mconfun and/or mcongrad not defined and matrix constraints present');
            end
            obj.mconfun=penm.mconfun;
            obj.mcongrad=penm.mcongrad;
          end
          if (obj.NANLN>0)
            if (isfield(penm,'mconlagrhess'))
              obj.mconlagrhess=penm.mconlagrhess;
            elseif (isfield(penm,'mconhess'))
              obj.mconhess=penm.mconhess;
            else
              error('Neither mconlagrhess nor mconhess is defined and nonlinear matrix constraints present');
            end
          end

          % generate Adep - is the current point good for it?
           if (isfield(penm,'Adep'))
               obj.Adep=penm.Adep;
           else
              obj.Adep=cell(obj.NANLN+obj.NALIN,1);
              for kuser=[1:obj.NANLN+obj.NALIN]
                  xh=rand(size(obj.x)); 
                  if length(obj.Y)>0
                  for iY=1:length(obj.Y), Yh{iY}=rand(size(obj.Y{iY}));end
                  else Yh{1}=[];end 
                  [Akx,obj.userdata] = obj.mconfun(xh, Yh, kuser, obj.userdata);
                  list=[];
                  for i=[1:obj.Nx+obj.NYnnz]                   
                      [Akdx, obj.userdata] = obj.mcongrad(xh,Yh,kuser,i,obj.userdata);
                      if (~isempty(Akdx)&& nnz(Akdx)>0)
                          list=[list,i];
                      end
                  end
                  obj.Adep{kuser}=list;
              end
           end

          % generate: *type


          % initialize penalty type lists for each constraints (bar/phi?)
          obj.setpentype(penm.lbxbar,penm.ubxbar,penm.lbYbar,penm.ubYbar);

        else
          error('ERR: provide a penm struct!');
        end
      else
        disp('WARNING: Empty object! Call with "penm" structure!');
      end
      obj.phase = 1;
    end

    % destructor
    function delete(obj)
      % close log file
      obj.logfile(0);
    end

    % whenever user changes 'opts', check their validity and copy through
    % to allopts; if after INIT phase (=not called from the constructor), 
    % report the change to the printer as well (TODO)
    function set.opts(obj, newopts)
      %disp('changing options?');
      % to accept the option it
      %  - must be already present in 'allopts' (==> must be in penlab.defopts())
      %  - consider only new values
      %  - only if they are allowed (in bounds etc.)

      %newopts

      % isstruct?? & nonempty?
      if (isempty(newopts) || ~isstruct(newopts) || isempty(fieldnames(newopts)))
        % restoring all to defautls
        % if wasn't empty & not init... to avoid the message while init
        obj.allopts = penlab.defopts(1);
        obj.opts = struct();
        obj.logfile(1);
        disp('Restoring all option settings to their defaults.');
      else

        names=fieldnames(newopts);
        discard=find(~isfield(penlab.defopts(1),names));
        if (~isempty(discard))
          str = sprintf(' %s',names{discard});
          fprintf('Warning: these options don''t exist and are ignored:%s\n',str);
        end

        %keep=find(isfield(obj.allopts,names) && (~isfield(obj.opts,names) || getfield(obj.opts,names)~=getfield(newopts,names)));
        keep=find(isfield(penlab.defopts(1),names));
        reopenlog=0;
        for i=keep'    % need a row vector
          defvalue=getfield(penlab.defopts(1),names{i});
          value=getfield(newopts,names{i});
          if (isfield(obj.opts, names{i}))
            origvalue=getfield(obj.opts,names{i});
          else
            origvalue=[];
          end
          oldvalue=getfield(obj.allopts,names{i});
          newvalue=oldvalue;

          % delete, new or change obj.opts.names{i} ??
          if (isempty(value) && ~isempty(origvalue))
            % remove this one from user options and restore defaults
            obj.opts = rmfield(obj.opts, names{i});
            obj.allopts = setfield(obj.allopts, names{i}, defvalue);
            newvalue=defvalue;
            disp(sprintf('option %s set back to defautls',names{i}));

          elseif (isempty(origvalue) || origvalue~=value)
            % add or change
            % check validity of the new value
            obj.opts=setfield(obj.opts, names{i}, value);
            obj.allopts=setfield(obj.allopts, names{i}, value);
            newvalue=value;
            %disp(sprintf('new option %s set to %g',names{i},value));
              % universal format to print?? value can be anything
          end

          % need to reopen the log file?
          if (strcmp(names{i},'out_filename') || ...
            strcmp(names{i},'outlev_file') && oldvalue*newvalue==0)
            reopenlog=1;
          end
        end
        if (reopenlog)
          obj.logfile(1);
        end
      end
    end

    % Keep an eye on changes of xall (internal variables) and update ticker
    % to set the rest as invalid; this is the only place where ticker should
    % be updated
    function set.xall(obj, xall)
      %disp('...Tick, xall updated');
      % TODO if ticker is maxint, reset all tickers
      obj.ticker=obj.ticker+1;
      obj.xall=xall;
    end

    % Update x & Y from xall if not already up-to-date
    function x=get.x(obj)
      if (obj.xYtck<obj.ticker)
        %disp('...update of x,Y initialized by x');
        obj.x=obj.xall(1:obj.Nx);
        obj.Y=obj.vec2Y(obj.xall(obj.Nx+1:end));
        obj.xYtck=obj.ticker;
      end
      x=obj.x;
    end

    % Update x & Y from xall if not already up-to-date
    function Y=get.Y(obj)
      if (obj.xYtck<obj.ticker)
        %disp('...update of x,Y initialized by Y');
        obj.x=obj.xall(1:obj.Nx);
        obj.Y=obj.vec2Y(obj.xall(obj.Nx+1:end));
        obj.xYtck=obj.ticker;
      end
      Y=obj.Y;
    end

    % changing starting x by user is not allowed during solving
    function set.xinit(obj,x)
      if (obj.phase==2)
        error('Changing x during computation is not allowed.');
      end

      %%%disp('...changing x starting point');
      if (length(x)~=obj.Nx)
        error('wrong dimension of x');
      end
      obj.xall(1:obj.Nx) = x;
      if (obj.phase==3)
        % throw away all results, TODO prehaps need to do something more?
        obj.phase=1;
      end
    end

    % changing starting Y (not allowed during solving)
    function set.Yinit(obj,Y)
      if (obj.phase==2)
        error('Changing Y during computation is not allowed.');
      end

      %%%disp('...changing Y starting point');
      % if Y is not complete, use current xall to fill in gaps
      obj.xall(obj.Nx+1:end)=obj.Y2vec(Y);
      if (obj.phase==3)
        % throw away all results, TODO prehaps need to do something more?
        obj.phase=1;
      end
    end


  end

  methods (Static)

    % default Problem name
    function dname = default_probname()
      dname = ['Pennon NLP-SDP problem ', datestr(now) ];
    end

    % default option settings (only values can be changed, new options
    % need to have a default value first)
    function dopts = default_opts()
      dopts = [];
      dopts.outlev = 2;
      dopts.outlev_file = 5;
      dopts.out_filename = 'penm_log.txt';
      dopts.user_prn = [];
      dopts.maxotiter = 100;
      dopts.maxiniter = 100;
      % ...
      % from pennon.m
      dopts.penalty_update = 0.5;       % PENALTY_UPDT
      dopts.penalty_update_bar = 0.3;   % PENALTY_UPDT_BAR
      dopts.max_outer_iter = 100;       % MAX_PBMITER
      dopts.outer_stop_limit = 1e-6;    % PBMALPHA
      dopts.kkt_stop_limit = 1e-4;      % KKTALPHA
      dopts.mlt_update =0.3;            % MU
      dopts.uinit = 1;                  % UINIT
      dopts.uinit_box = 1;              % UINIT_BOX
      dopts.uinit_eq = 0;               % UINIT_EQ
      dopts.umin = 1e-10;               % UMIN
      dopts.pinit = 1;                  % PINIT
      dopts.pinit_bar = 1;              % PINIT_BAR
      dopts.usebarrier = 0;             % USEBARRIER
      % from unconstr_min.m
      dopts.max_inner_iter = 100;       % MAX_MITER
      dopts.inner_stop_limit = 1e-2;    % ALPHA
      dopts.unc_dir_stop_limit = 1e-2;  % TOL_DIR
      dopts.unc_solver = 0;             % solver
      dopts.unc_linesearch = 3;         % linesearch
      % from eqconstr_min.m
      dopts.eq_dir_stop_limit = 1e-2;   % TOL_DIR
      dopts.eq_solver = 0;              % solver
      dopts.eq_linesearch = 3;          % linesearch
      dopts.eq_solver_warn_max = 4;     % solver_warn_max
      dopts.ls_short_max = 3;           % ls_short_max
      dopts.min_recover_strategy = 0;   % recover_strategy
      dopts.min_recover_max = 3;        % recover_max
      % from phi2.m
      dopts.phi_R = -0.5;               % R_default

      % linesearchs - ls_armijo.m
      dopts.max_ls_iter = 20;   % max tries before LS fails
      dopts.max_lseq_iter = 20;   % same for LS for equality constrained problems
      dopts.armijo_eps = 1e-2;  % when is armijo step satisfactory? P(alp) - P(0) <= eps*alp*P'(0)

      % solve_simple_chol.m, solkvekkt_ldl.m, solvekkt_lu.m
      dopts.pert_update = 2.;  % known aka LMUPDATE, multiplier of the lambda-perturbation factor
      dopts.pert_min = 1e-6;   % LMLOW, minimal (starting) perturbation
      dopts.pert_try_max = 50; % max number of attempts to successfully perturbate a matrix
      dopts.pert_faster = 1;   % use the last known negative curvature vector to determine perturbation

      % solve_simple_chol.m
      dopts.chol_ordering = 1; % use symamd for sparse matrices before Cholesky factor.?

      % solvers
      dopts.luk3_diag = 1;  % diagonal of the (2,2)-block in Luksan 3 preconditioner
      %%% different stuff from previous 'popt' structure %%%
      % equality constrained/unconstrained minimization
      % - eqconstr_min.m, unconstr_min.m
      %dopts.eq_dir_max_prec = 1e-6; % upper limit to the precision of a solution, do not demand a better (absolute) precision than this one; set 0 to turn it off

      % tracing of one inner loop
      %dopts.trace = 0;   % turn on tracing?
      %dopts.trace_outer_iter = 1; % which outer iteration to trace? (1~the whole first iter ~ from the very beginning)
      %dopts.trace_filename='trace_point.dcf'; % name of the dcf file if used

    end

  end


end

