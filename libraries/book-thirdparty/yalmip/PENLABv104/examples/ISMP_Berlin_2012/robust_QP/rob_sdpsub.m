function [penm] = rob_sdpsub(A,b,unc,xx)
% 
% call:
%   penm=ex3_define_by_anonymous();    % to define structure 
%   prob=penlab(penm);    % to convert the structure and initialize the problem
%   prob.opts....=...;    % to change option settings if desired
%   prob.solve();         % to start the solver
%   prob.x                % to retrieve the final point (solution)

n = length(A);
[xxvec,idiag] = packmat(xx*xx');
gg=zeros(n,idiag(end)); for i=1:n,gg(i,idiag(i))=1;end
m=idiag(end);

  penm = [];

  penm.probname = 'rob_sdpsub';
  penm.comment = 'linear SDP subproblem of robust QP';

  penm.NY = 1;
  penm.Y{1} = ones(n,n);
  penm.lbY = -1;
  penm.ubY = 1;

    penm.NgLIN = n;
    penm.lbg = zeros(n,1);
    penm.ubg = zeros(n,1);
  
 penm.NALIN = 0;
 penm.lbA=0;
 penm.ubg = zeros(n,1);

  % define all call-backs as anonymous functions
  penm.objfun  = @(x,Y,userdata) deal(-xx'*(A+unc.*Y{1})*xx, userdata);
  penm.objgrad = @(x,Y,userdata) deal(-unc.*xxvec, userdata);
  penm.objhess = @(x,Y,userdata) deal(zeros(m,m), userdata);
  
   penm.confun  = @(x,Y,userdata) deal(diag(Y{1}), userdata);
   penm.congrad = @(x,Y,userdata) deal(gg', userdata);
  
  penm.mconfun  = @(x,Y,k,userdata) deal(A+unc.*Y{k}, userdata);
  penm.mcongrad = @(x,Y,k,i,userdata) deal(unc.*sparse(ones(n,n)), userdata);