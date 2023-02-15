function [Akx, userdata] = bmi_mconfun(x,Y,k,userdata)
% evaluate A_k(x) based on userdata, k denotes a block number
% userdata is a preproccessed structure obtained from the user

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

% note that we aim for A_k(x)>=0 (pos. definite) so it is evaluated
% exactly how it is stored
%   A_k(x) = sum_i x^(multi-index(i))*Q_i

  if (k>=1 && k<=userdata.NA)
    % matrix constraints were shuffled to keep nonlinear first,
    % return the right block as solver sees it
    k_usr = userdata.mconorder(k);
    Akx=eval_akx([1.0;x], userdata.mcon{k_usr}.dim, userdata.mcon{k_usr}.midx, userdata.mcon{k_usr}.Q);
  else
    Akx=[];
  end
end


%%%%%%%%%%
% evaluate a single matrix constraint
%   xex .... extended x [1; normal x] to be able to easily address the issue
%       with 0 index in the multi-index 
%   dimQ ... dimension of the matrix constraint 
%   midx ... multi-indices nOrder x nMat with values 0..Nx where 0 represents
%       an element not present, for example:
%          midx(:,5)=[1;4]
%       means that Q{5} is a matrix which has weight x_1*x_4 and max. order
%       in the whole matrix constraint is 2;
%          midx(:,1)=[0;0]
%       shows that Q{1} is the absolute term in the matrix constraint
%   Q{nMat} . cell array defining sparse matrices for each of the multi-indices
%
function [Akx] = eval_akx(xex,dimQ,midx,Q)

  Akx=sparse(dimQ,dimQ);
  [nOrder, nMat] = size(midx);

  if (nOrder==1)
    weights = xex(midx+1);
  else
    % bilinear/quadratic terms
    % compute the right weight for every Q{imat} matrix given by 
    %    x_i*x_j
    % where i,j are given in the multiindex midx(:,imat).
    weights = prod(xex(midx+1),1);
  end

  for imat=1:nMat
    Akx = Akx + weights(imat).*Q{imat};
  end
end

