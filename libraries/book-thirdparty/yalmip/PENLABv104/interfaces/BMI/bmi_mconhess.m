function [Akddx, userdata] = bmi_mconhess(x,Y,k,i,j,userdata)
% Compute derivative: d2/dx_idx_j A_k(x) based on the data from userdata

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  if (k>=1 && k<=userdata.NA)
    % matrix constraints were shuffled to keep nonlinear first,
    % return the right block as solver sees it
    k_usr = userdata.mconorder(k);
    Akddx=eval_akddx([1.0;x], i, j, userdata.mcon{k_usr}.dim, userdata.mcon{k_usr}.midx, userdata.mcon{k_usr}.Q);
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
%          midx(:,5)=[1;0]
%       means that Q{5} is a matrix which has weight x_1 and max. order
%       in the whole matrix constraint is 2;
%          midx(:,1)=[0;0;0]
%       shows that Q{1} is the absolute term in the matrix constraint
%   Q{nMat} . cell array defining sparse matrices for each of the multi-indices
%
% Note, need to return empty array (not empty matrix) if there is no dependency.
%
function [Akddx] = eval_akddx(xex,i,j,dimQ,midx,Q)

  % find out among bilinear/quadratic terms which (if any) Q{imat} matrices 
  % have x_i,x_j in their multiindex
  % Note, it is necessary to distinquish if i==j / i~=j!
  if (i==j)
    % looking for quadratic terms in x_i
    xi_order=sum(midx==i,1);
    list=find(xi_order==2);

    if (isempty(list))
      Akddx=[];
    else
      % sum all Q matrices from the list with coeficient 2 
      % (x_i^2 * Q -> 2 * Q); in theory, there should be just one such Q

      Akddx = 2*Q{list(1)};

      for imat=list(2:end)
        Akddx = Akddx + 2*Q{imat};
      end
    end
  else
    % looking for multi-indices with both x_i and x_j
    listi=find(any(midx==i,1));
    listij=find(any(midx(:,listi)==j,1));
    list=listi(listij);

    % alternatively (any faster?):
    % isi = midx==i
    % isj = midx==j
    % list = find(isi(1,:) & isj(2,:) | isi(2,:) & isj(1,:))

    if (isempty(list))
      Akddx=[];
    else
      % sum all Q matrices from the list (x_i*x_j * Q -> Q)

      Akddx = Q{list(1)};

      for imat=list(2:end)
        Akddx = Akddx + Q{imat};
      end
    end
  end

end

