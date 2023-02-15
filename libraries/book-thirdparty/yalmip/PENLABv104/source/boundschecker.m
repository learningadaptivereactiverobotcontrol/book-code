% Checks bounds that the dimension is correct and make sense w.r.t. the
% settings (equality allowed? unconstrained allowed?) and create mapping from
%   lb <= sth <= ub    to    constr <=0   or   constr==0
%   lb_i <= sth_i    becomes     constr_k=lb-sth_i <=0   
%         with  ineqmap(k)=i, ineqmlt(k)=-1, ineqshift(k)=lb_i   i.e.
%         constr_k = ineqshift(k) + ineqmlt(k)*sth_{ineqmap(k)}
%   if(equalok>0) ... equalities are allowed and treated as equalities
%   if(equalok<0) ... equalities are allowed and treated as 2 inequalities
%            (temporary solution for box equality constriants on matrices)
%   if(unconstrok) ... unconstrained row (lb=-inf,ub=inf) is allowed
%            (and won't be included in any map, e.g. unbounded variable)
%   filterout ... array of indicies 1..dim which shouldn't be considered at all
%      (e.g., constraints on empty matrix variables)
function [ineqmap, ineqmlt, ineqshift, eqmap, eqshift]=boundschecker(dim,lb,ub,equalok,unconstrok,filterout)

  ineqmap=[];
  ineqmlt=[];
  ineqshift=[];
  eqmap=[];
  eqshift=[];

  if (dim>0)
    if (~isvector(lb) || ~isvector(ub) || ...
        (~isscalar(lb) && length(lb)~=dim) || ...
        (~isscalar(ub) && length(ub)~=dim))
      error('Error in dim or shape lb or ub');
      return;
    end
    if (isscalar(lb))
      lb=lb.*ones(dim,1);
    end
    if (isscalar(ub))
      ub=ub.*ones(dim,1);
    end
    if (size(lb,1)<size(lb,2))
      lb=lb';
    end
    if (size(ub,1)<size(ub,2))
      ub=ub';
    end

    % what if lb/ub row/column vector? 
    if (any(lb==Inf | ub==-Inf | lb>ub))
      error('Error in values lb and/or ub');
      return;
    end

    Nineq=0;
    Neq=0;

    indices=setdiff([1:dim],filterout);
    for i=indices
      if (lb(i)==ub(i))
        if (equalok>0)
          % as equality
          Neq=Neq+1;
          eqmap(Neq)=i;
          eqshift(Neq)=-lb(i);
          %eqmap = [eqmap; i];
          %eqshift = [eqshift; -lb(i)];
        elseif (equalok<0)
          % as two inequalitites
          Nineq=Nineq+1;
          ineqmap(Nineq)=i;
          ineqmlt(Nineq)=-1;
          ineqshift(Nineq)=lb(i);

          Nineq=Nineq+1;
          ineqmap(Nineq)=i;
          ineqmlt(Nineq)=1;
          ineqshift(Nineq)=-ub(i);
        else
          error('equality here and forbidden!');
          return;
        end
      else
        if (lb(i)>-Inf)
          Nineq=Nineq+1;
          ineqmap(Nineq)=i;
          ineqmlt(Nineq)=-1;
          ineqshift(Nineq)=lb(i);
        end
        if (ub(i)<Inf)
          Nineq=Nineq+1;
          ineqmap(Nineq)=i;
          ineqmlt(Nineq)=1;
          ineqshift(Nineq)=-ub(i);
        elseif (lb(i)==-Inf && ~unconstrok)
          error('err: unconstr & not allowed!');
          return;
        end
      end
    end
    % transpose all to get column vectors instead of rows
    ineqmap=ineqmap';
    ineqmlt=ineqmlt';
    ineqshift=ineqshift';
    eqmap=eqmap';
    eqshift=eqshift';
  end

