function [Akdx, userdata] = bmi_mcongrad(x,Y,k,i,userdata)
% Compute derivative: d/dx_i A_k(x) based on the data from userdata

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  if (k>=1 && k<=userdata.NA)
    % matrix constraints were shuffled to keep nonlinear first,
    % return the right block as solver sees it
    k_usr = userdata.mconorder(k);
    Akdx=eval_akdx([1.0;x], i, userdata.mcon{k_usr}.dim, userdata.mcon{k_usr}.midx, userdata.mcon{k_usr}.Q);
  else
    Akdx=[];
  end
end

%%%%%%%%%%
% evaluate a first derivative of a single matrix constraint
%   xex .... extended x [1; normal x] to be able to easily address the issue
%       with 0 index in the multi-index 
%   i ...... index of the derivative (d/dx_i)
%   dimQ ... dimension of the matrix constraint 
%   midx ... multi-indices nOrder x nMat with values 0..Nx where 0 represents
%       an element not present, for example:
%          midx(:,5)=[1;0]
%       means that Q{5} is a matrix which has weight x_1 and max. order
%       in the whole matrix constraint is 2;
%          midx(:,1)=[0;0]
%       shows that Q{1} is the absolute term in the matrix constraint
%   Q{nMat} . cell array defining sparse matrices for each of the multi-indices
%
% Note, need to return empty array (not empty matrix) if there is no dependency.
%
function [Akdx] = eval_akdx(xex,i,dimQ,midx,Q)

  [nOrder, nMat] = size(midx);

  if (nOrder==1)
    % linear terms only
    list=find(midx==i);

    if (isempty(list))
      % no dependency on x_i
      Akdx=[];
    else
      % sum all the matrices which had x_i (in theory should be just one)
      Akdx=Q{list(1)};

      for imat=list(2:end)
        Akdx = Akdx + Q{imat};
      end
    end

  else
    % bilinear/quadratic terms
    % find out which (if any) Q{imat} matrices have x_i in their multiindex
    % (i.e., the matrix constraints which depend on x_i)
    isxi = midx==i;
    list = find(any(isxi,1));

    if (isempty(list))
      % no dependency on x_i
      Akdx=[];
    else
      len=length(list);
      Akdx=sparse(dimQ,dimQ);

      % reduce ourselves only to 'list' subset
      isxi_r = isxi(:,list);
      midx_r = midx(:,list)+1;
      % compute appropriate weights, i.e.,
      %   x_i*x_j -> x_j;  x_i*x_i -> 2*x_i
      weights = zeros(len,1);
      % grrr, troubles with empty matrices, need to add checks
      if (any(isxi_r(1,:)))
        weights(isxi_r(1,:)) = xex(midx_r(2,isxi_r(1,:)));
      end
      if (any(isxi_r(2,:)))
        weights(isxi_r(2,:)) = weights(isxi_r(2,:)) + xex(midx_r(1,isxi_r(2,:)));
      end

      for imat=1:len
        Akdx = Akdx + weights(imat).*Q{list(imat)};
      end
    end
  end
end

