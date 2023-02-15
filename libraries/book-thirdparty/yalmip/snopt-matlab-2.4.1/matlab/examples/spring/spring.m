function [x,F,xmul,Fmul,INFO] = spring()


fprintf('\n====================================== ');
fprintf('\nspring: Solving SPRING using SNOPT ... ');
fprintf('\n====================================== ');

snscreen on;
snprint('spring.out');

specs = which('spring.spc');
snspec (specs);


[x,xlow,xupp,xmul,xstate, ...
   Flow,Fupp,Fmul,Fstate, ...
 ObjAdd,ObjRow] = springData;

[x,F,INFO,xmul,Fmul]= snopt( x, xlow, xupp, xmul, xstate,  ...
                             Flow, Fupp, Fmul, Fstate,     ...
                             @springFun, ObjAdd, ObjRow);

snprint off; % Closes the file and empties the print buffer
snend;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x,xlow,xupp,xmul,xstate, ...
          Flow,Fupp,Fmul,Fstate,   ...
          ObjAdd,ObjRow] = springData()

    T      = 100;

    n      = 3*T + 2;
    nCon   = 2*T;
    nF     = nCon + 1;

    Obj    = nF;
    ObjRow = Obj;

    ObjAdd = 0;

    x		= zeros(n,1);
    xstate	= zeros(n,1);
    xmul	= zeros(n,1);
    xlow	= -inf*ones(n,1);
    xupp	=  inf*ones(n,1);

    F		= zeros(nF,1);
    Fstate	= zeros(nF,1);
    Fmul	= zeros(nF,1);
    Flow	= -inf*ones(nF,1);
    Fupp	=  inf*ones(nF,1);

    jy0 = 1;  jx0 = jy0 + T + 1; ju0 = jx0 + T + 1;

    for jt = 0:T
      jy = jy0 + jt; jx = jx0 + jt;

      xlow(jx) = -inf; xupp(jx)   = inf;
      x(jx)    = 0;    xstate(jx) = 3;

      xlow(jy) = -1; xupp(jy)   = inf;
      x(jy)    = -1; xstate(jy) = 0;

      if jt < T,
	ju = ju0 + jt;

	xlow(ju) = -.2; xupp(ju)   = .2;
	x(ju)    = 0;   xstate(ju) = 3;
      end
    end


    % Boundary conditions
    xlow(jy0)   = 0;  xupp(jy0)   = 0;
    xlow(jy0+T) = 0;  xupp(jy0+T) = 0;  x(jy0+T) = 0;
    xlow(jx0)   = 10; xupp(jx0+T) = 10; x(jx0+T) = 10;

    % Bounds on F
    Flow(1:nCon) = 0;    Fupp(1:nCon) = 0;   Fmul(1:nCon) = 0;
    Flow(ObjRow) = -inf; Fupp(ObjRow) = inf; Fmul(ObjRow) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
