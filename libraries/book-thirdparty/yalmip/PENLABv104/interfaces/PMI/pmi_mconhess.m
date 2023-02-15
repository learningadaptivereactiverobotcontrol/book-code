function [Akddx, userdata] = pmi_mconhess(x,Y,k,i,j,userdata)
% Compute derivative: d2/dx_idx_j A_k(x) based on the data from userdata

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  if (k>=1 && k<=userdata.NA)
    Akddx=eval_akddx([1.0;x], i, j, userdata.mcon{k}.dim, userdata.mcon{k}.midx, userdata.mcon{k}.Q);
  else
    Akddx=[];
  end
end

%%%%%%%%%%
% evaluate a second derivative of a single matrix constraint
%   xex .... extended x [1; normal x] to be able to easily address the issue
%       with 0 index in the multi-index 
%   i,j .... indeces of the derivative (d/dx_i d/dx_j)
%   dimQ ... dimension of the matrix constraint 
%   midx ... multi-indices nOrder x nMat with values 0..Nx where 0 represents
%       an element not present, for example:
%          midx(:,5)=[1;4;0]
%       means that Q{5} is a matrix which has weight x_1*x_4 and max. order
%       in the whole matrix constraint is 3;
%          midx(:,1)=[0;0;0]
%       shows that Q{1} is the absolute term in the matrix constraint
%   Q{nMat} . cell array defining sparse matrices for each of the multi-indices
%
% Note, need to return empty array (not empty matrix) if there is no dependency.
%
function [Akddx] = eval_akddx(xex,i,j,dimQ,midx,Q)

  % find out which (if any) Q{imat} matrices have x_i,x_j in their multiindex
  % (i.e., the matrix constraints which depend on x_i,x_j) and their order
  % Note, it is necessary to distinquish if i==j / i~=j!
  if (i==j)
    % looking for at least quadratic terms in x_i
    xi_order=sum(midx==i,1);
    list=find(xi_order>=2);

    if (~isempty(list))
      % compute the right weight for every Q{imat} matrix given by 
      %    xi_order*(xi_order-1) * (x_i1*x_i2*...*x_imax) / x_i/x_i
      % where i1,i2,...,imax is the multiindex midx(:,imat) with
      % respect to d/dx_i. Remove xex(i+1) and add its contribution in mlt.
      xi=xex(i+1);
      xex(i+1)=1;
      xio=xi_order(list);
      mlt=xio.*(xio-1).*xi.^(xio-2);
      weights = prod(xex(midx(:,list)+1),1).*mlt;
    end
  else
    % looking for multi-indices with both x_i and x_j
    listi=find(any(midx==i,1));
    listij=find(any(midx(:,listi)==j,1));
    list=listi(listij);

    if (~isempty(list))
      % compute the right weight for every Q{imat} matrix given by 
      %    xi_order*xj_order * (x_i1*x_i2*...*x_imax) / x_i/x_j
      % where i1,i2,...,imax is the multiindex midx(:,imat) with
      % respect to d/dx_i. Similarly, remove xi, xj.

      xi=xex(i+1);
      xj=xex(j+1);
      xex(i+1)=1;
      xex(j+1)=1;
      xi_order=sum(midx(:,list)==i,1);
      xj_order=sum(midx(:,list)==j,1);
      mlt=xi_order.*xj_order.*xi.^(xi_order-1).*xj.^(xj_order-1);

      weights = prod(xex(midx(:,list)+1),1).*mlt;
    end
  end

  if (isempty(list))
    % no dependency on x_i*x_j
    Akddx=[];
  else
    Akddx=sparse(dimQ,dimQ);

    len=length(list);
    for imat=1:len
      Akddx = Akddx + weights(imat).*Q{list(imat)};
    end
  end
end

