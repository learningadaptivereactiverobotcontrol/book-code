function [Akdx, userdata] = pmi_mcongrad(x,Y,k,i,userdata)
% Compute derivative: d/dx_i A_k(x) based on the data from userdata

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  if (k>=1 && k<=userdata.NA)
    Akdx=eval_akdx([1.0;x], i, userdata.mcon{k}.dim, userdata.mcon{k}.midx, userdata.mcon{k}.Q);
  else
    Akdx=[];
  end
end

function [Akdx] = eval_akdx(xex,i,dimQ,midx,Q)
%%%%%%%%%%
% evaluate a first derivative of a single matrix constraint
%   xex .... extended x [1; normal x] to be able to easily address the issue
%       with 0 index in the multi-index 
%   i ...... index of the derivative (d/dx_i)
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
  % find out which (if any) Q{imat} matrices have x_i in their multiindex
  % (i.e., the matrix constraints which depend on x_i) and the order
  xi_order=sum(midx==i,1);
  list=find(xi_order);

  if (isempty(list))
    % no dependency on x_i
    Akdx=[];
  else
    len=length(list);
    Akdx=sparse(dimQ,dimQ);

    % compute the right weight for every Q{imat} matrix given by 
    %    xi_order * (x_i1*x_i2*...*x_imax) / x_i
    % where i1,i2,...,imax is the multiindex midx(:,imat) with
    % respect to d/dx_i. But it cannot be computed this way because
    % of the case with xi=0.
    %weights = prod(xex(midx(:,list)+1),1).*xi_order(list)./xex(i+1);
    
    % --> rather remove all xi from the product and then multiply it later
    % in mlt we should have: xi_order*xi^(xi_order-1)
    xi=xex(i+1);
    xex(i+1)=1;
    xio=xi_order(list);
    %iii=find(xio>=2);
    %mlt=ones(1,len);
    %mlt(iii)=xio(iii).*(xi.^(xio(iii)-1));
    mlt=xio.*xi.^(xio-1);
    
    weights = prod(xex(midx(:,list)+1),1).*mlt;

    for imat=1:len
      Akdx = Akdx + weights(imat).*Q{list(imat)};
    end
  end
end

