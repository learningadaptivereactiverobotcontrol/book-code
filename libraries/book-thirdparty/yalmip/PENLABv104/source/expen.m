% example pennon structure to play with
% call penm=expen & problem=penlab(penm)
function penm=expen()

  penm = [];

  % [optional] problem name and a comment (e.g. source)
  % just for logging
  penm.probname = 'Example Problem 1';
  penm.comment = 'bla bla comment';

  %% [optional] userdata
  % will get always pass in and from any callback function, can be anything
  % if not set, an empty array will be used instead
  penm.userdata = [];

  % [optional] option settings different from defaults
  penm.opts=[];

  %% define decision variables
  % number of (ordinary) decision variables
  % If the field is not present, Nx=0; lower/upper bounds for box constraints
  % will be ignored.
  penm.Nx = 7;
  % number of matrix variables, should be the same as length(Y)
  % If the field is not present, NY=0; thus Y{:}, bounds etc. will be ignored.
  penm.NY = 3;
  % cell array of length 'NY' with a nonzero pattern of each of the matrix 
  % variables. Each matrix will be symmetrized and all its nonzeros in one 
  % triangle will be considered as variables, the rest as constant zeros.
  % Whenever array Y{} is present in the callbacks, it will be symmetrized.
  % All (element-)variables will be counted with indicies starting with (Nx+1),
  % the order is Y{1}, Y{2}, ..., Y{NY} and in each matrix lower (symmetrized)
  % triangle column-wise. If any (derivative) callback needs derivatives
  % with respect to 'k' (k>Nx), it means it's one of the variables in one
  % of the matrices. A routine '...' can generate back the matrix and the exact
  % position of the variable ...
  penm.Y{1} = ones(3);
  penm.Y{2} = spones(sprandsym(5,0.3));
  penm.Y{3} = spones(sprand(6,6,0.2));

  %% decision variables box constraints
  % box constraints (lower/upper bounds) on the variables, lbx <= x <= ubx
  % if lbx==-Inf, it is not considered; similarly if ubx=Inf so, e.g.,
  % by setting lbx=-Inf, ubx=Inf the variable is unbounded.
  % length of each array should be Nx
  % if one or both are missing, 'x' is considered as unconstrained from that
  % side. Any ubx>lbx is considered as error. If lbx==ubx the variable will
  % be fixed and won't have further effect.
  penm.lbx = 0;
  %penm.lbx = [-Inf, -Inf, -1, 0, -3, -6, -77];
  penm.ubx = [10, Inf, Inf, 4, 66, 0.1, Inf];

  % lower, upper bounds on Y variable elements?
  % if not present, unconstrained
  % penm.lbYx, ubYx ? or not at all? It can be always defined as one of the g()
  % constraints

  % lower and upper bound on matrices Y (as matrix inequalities, i.e. eigenvals)
  % if fixed... what to do? Might even not have solution...
  % length NY
  penm.lbY = [0, -Inf, 1];
  penm.ubY = [Inf, 0, 10];

  % [optional] box constraints on the elements of the matrix variables
  % only elements matchin patterns of Y{} will be taken into account
  % if the matrix is empty ([]) it is automatically considered as +/-Inf 
  % accordingly
  penm.lbYx{1} = - magic(3);
  penm.ubYx{1} = magic(3);
  penm.lbYx{2} = sparse(5,5);
  penm.ubYx{3} = 100;

  %% starting point
  % ideally feasible, might get adjusted, optional, if not set will be computed
  % somehow. With the matrices, it will accept only these nnz which where in 
  % Y pattern. Symmetric or automatically lower triangle???
  penm.xinit=rand(penm.Nx,1);
  %penm.Yinit{1} {2} {3}=....;

  %% linear and nonlinear function constraints
  % nonlinear constraints, first NLN , then LIN
  penm.NgNLN = 2;
  penm.NgLIN = 1;

  % nonlinear and linear matrix constraints
  penm.NANLN = 0;
  penm.NALIN = 1;

  % lower/upper bounds ... equalities, must be defined at least on one side
  % (e.g. not unconstrained constraints ;-)
  penm.lbg = [-Inf, 0, 5];
  penm.ubg = [0, Inf, 5];
  penm.lbA = [0];
  penm.ubA = [Inf];

  %% call back functions to evaluate objective function, function constraints
  % and matrix constraint and their derivatives. First go NLN then LIN.
  %penm.objfun = ...
  % [fx, userdata] = objfun(x,Y,userdata)   % returns a number
  % [fdx, userdata] = objgrad(x,Y,userdata) % returns a (sparse) vector
     % w.r.t. all variables (even matrix ones)
  % objfun, objgrad, objhess
  % confun, congrad, conhess
  % mconfun, mcongrad, mconhess
  % alternatively objhess + conhess -> lagrhess

  %[fx, userdata] = objfun(x,Y,userdata)   % returns a number

  %[fdx, userdata] = objgrad(x,Y,userdata) % returns a (possibly sparse) vector
     % w.r.t. all variables (even matrix ones), if objfun doesn't depend on Y
     % it can be just Nx long, otherwise Nx+NYnnz


