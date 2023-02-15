function [x,F,xmul,Fmul,INFO] = hs116()

snprint('hs116.out');

hs116.spc = which('hs116.spc');
snspec (hs116.spc);

snseti('Major Iteration limit', 250);

[x,xlow,xupp,xmul,xstate, ...
   Flow,Fupp,Fmul,Fstate, ...
 ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar] = hs116data;

[x,F,INFO,xmul,Fmul]= snopt( x, xlow, xupp, xmul, xstate,  ...
                             Flow, Fupp, Fmul, Fstate,     ...
                             @hs116userfun, ObjAdd, ObjRow, ...
                             A, iAfun, jAvar, iGfun, jGvar );

snprint off; % Closes the file and empties the print buffer
snend;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x,xlow,xupp,xmul,xstate, ...
          Flow,Fupp,Fmul,Fstate,   ...
          ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar] = hs116data()
%     hs116data defines problem HS116.
%
%     Minimize      x(11) + x(12) + x(13)
%     subject to    many constraints
    n      = 13;
    neF    = 15;
    Obj    = 5;
    ObjRow = Obj;

    %% parameters
    a = 0.002;
    b = 1.262626;
    c = 1.231059;
    d = 0.03475;
    e = 0.975;
    f = 0.00975;
    % HS Solution
    x = [   0.80377
            0.89999
            0.97095
            0.10000
            0.19081
            0.46057
          574.07758
           74.07758
          500.01615
            0.10000
           20.23309
           77.34768
            0.00673];
    %% initial x
    x = [   0.5;
            0.8;
            0.9;
            0.1;
            0.14;
            0.5;
            489;
             80;
            650;
            450;
            150;
            150;
            150   ];
    %% lower bounds on x
    xlow = [    0.1;
                0.1;
                0.1;
                1e-4;
                0.1;
                0.1;
                0.1;
                0.1;
                500;
                0.1;
                1;
                1e-4;
                1e-4    ];
    %% upper bounds on x
    xupp = [    1  ;
                1  ;
                1  ;
                0.1;
                0.9;
                0.9;
             1000;
             1000;
             1000;
              500;
              150;
              150;
              150   ];

    xstate = zeros(n,1);
    xmul   = zeros(n,1);

    iAfun  = []; jAvar = []; A = [];
    ObjAdd = 0;

    %% Bounds on F

    Flow      = zeros(neF,1); Fupp      = Inf*ones(neF,1);
    Flow(Obj) = -Inf;         Fupp(Obj) = Inf;
    Flow(3)   = -Inf;         Fupp(3)   = 1;
    Flow(4)   =   50;         Fupp(4)   = 250;
    Flow(13)  = -Inf;         Fupp(13)  = 1;
    Flow(15)  =  0.9;         Fupp(15)  = Inf;

    Fmul    = zeros(neF,1);
    Fstate  = zeros(neF,1);

G = [   1,  2, -1;
        1,  3,  1;
        2,  1, -1;
        2,  2,  1;
        3,  7,  a;
        3,  8, -a;
        4, 11,  1;
        4, 12,  1;
        4, 13,  1;
      Obj, 11,  1;
      Obj, 12,  1;
      Obj, 13,  1;
        6,  3,  c*x(10);
        6, 10, -b + c*x(3);
        6, 13,  1;
        7,  2, -d - e*x(5) + 2*f*x(2);
        7,  5,  1 - e*x(2);
        8,  3, -d - e*x(6) + 2*f*x(3);
        8,  6,  1 - e*x(3);
        9,  1, -d - e*x(4) + 2*f*x(1);
        9,  4,  1 - e*x(1);
       10,  2,  c*x(9);
       10,  9, -b + c*x(2);
       10, 12,  1;
       11,  1,  c*x(8);
       11,  8, -b + c*x(1);
       11, 11,  1;
       12,  1, -x(8);
       12,  4, -x(7) + x(8);
       12,  5,  x(7);
       12,  7,  x(5) - x(4);
       12,  8, -x(1) + x(4);
       13,  1, -a*x(8);
       13,  2,  a*x(9);
       13,  5, -1 + a*x(8);
       13,  6,  1 - a*x(9);
       13,  8,  a*(x(5) - x(1));
       13,  9,  a*(x(2) - x(6));
       14,  2, -500 + x(9) + x(10);
       14,  3, -x(10);
       14,  6,  500 - x(9);
       14,  9,  x(2) - x(6);
       14, 10, -x(3) + x(2);
       15,  2,  1 - a*x(10);
       15,  3,  a*x(10);
       15, 10, -a*(x(2) - x(3))    ];
    iGfun = G(:,1); jGvar = G(:,2);
