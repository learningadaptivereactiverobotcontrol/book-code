function [Akddx, userdata] = bmi_mconlagrhess(x,Y,k,Umlt,userdata)
% Compute 2nd derivative contribution of matrix constraint A_k(x)
% to the Augmented Lagrangian based on the data from userdata: 
%     (Akddx)_ij = trace (d2/dx_idx_j A_k(x),Umlt)
% Thus, Akddx should be (sparse or dense) matrix Nx x Nx (note that
% Y is not used in BMI)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  if (k>=1 && k<=userdata.NA)
    % matrix constraints were shuffled to keep nonlinear first,
    % return the right block as solver sees it
    k_usr = userdata.mconorder(k);
    Akddx=eval_akddx(x, Umlt, userdata.mcon{k_usr}.dim, userdata.mcon{k_usr}.midx, userdata.mcon{k_usr}.Q);
  else
    Akddx=[];
  end
end

%%%%%%%%%%
% evaluate all second derivatives of a single matrix constraint
%   x ...... x point to evaluate
%   Umlt ... Lagrangian multiplier
%   dimQ ... dimension of the matrix constraint 
%   midx ... multi-indices nOrder x nMat with values 0..Nx where 0 represents
%       an element not present, for example:
%          midx(:,5)=[1;0]
%       means that Q{5} is a matrix which has weight x_1 and max. order
%       in the whole matrix constraint is 2;
%          midx(:,1)=[0;0;0]
%       shows that Q{1} is the absolute term in the matrix constraint
%   Q{nMat} . cell array defining sparse matrices for each of the multi-indices
%
% Note, need to return empty array (not empty matrix) if there is no dependency.
%
function [Akddx] = eval_akddx(x,Umlt,dimQ,midx,Q)

  % midx can have at most order 2

  % find out bilinear/quadratic terms to know which (if any) Q{imat} to use
  list=find(sum(midx~=0,1) > 1);
  if (isempty(list))
    Akddx=[];
  else
    Nx = length(x);
    len=length(list);
    traces = zeros(1,len);
    for i = 1:len
      traces(i)=trace(Q{list(i)}*Umlt);
      %traces(i)=Umlt(:)'*Q{list(i)}(:);
    end
    Akddx = sparse(midx(1,list),midx(2,list),traces,Nx,Nx,2*len);
    % symmetrize: (i,j) derivatives needs to be reflected in 
    % both (i,j) and (j,i) elements and diagonal elements (i,i) should
    % have weight 2
    Akddx = Akddx + Akddx';
  end

end

