function [hgx, data] = fmo_conhess(x, Y, k, data)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

AA = data.AA;
d = data.d;
R = data.R;

tab=[1 2 4 3 5 6];

nelem = length(Y);
if k==1
    nnz = nelem*6*(nelem*6+1)/2;
    col = zeros(1,nnz);
    row = zeros(1,nnz);
    val = zeros(1,nnz);
    
    idx = 1;
    for ii=1:nelem
        for k=1:6
            Aikd(:,ii,k) = AA{ii,k}*d;
        end
        for k=1:6
            kk = tab(k);
            di = R\(R'\Aikd(:,ii,kk));
            for jj=1:ii-1
                for l=[1 2 4 3 5 6]
                    val(idx) = di'*Aikd(:,jj,l);
                    col(idx) = (ii-1)*6+kk+1;
                    row(idx) = (jj-1)*6+l+1;
                    idx = idx + 1;
                end
            end
            for ll=1:k
                l=tab(ll);
                val(idx) = Aikd(:,ii,l)'*di;
                col(idx) = (ii-1)*6+kk+1;
                row(idx) = (ii-1)*6+l+1;
                idx = idx + 1;
            end
        end
    end
    val = 2.*val;
%         idx = 1;
%     for ii=1:nelem
%         for k=1:6
%             kk = tab(k);
%             di = R\(R'\(AA{ii,kk}*d));
%             for jj=1:ii-1
%                 for l=[1 2 4 3 5 6]
%                     val(idx) = 2*d'*AA{jj,l}*di;
%                     col(idx) = (ii-1)*6+kk+1;
%                     row(idx) = (jj-1)*6+l+1;
%                     idx = idx + 1;
%                 end
%             end
%             for ll=1:k
%                 l=tab(ll);
%                 val(idx) = 2*d'*AA{ii,l}*di;
%                 col(idx) = (ii-1)*6+kk+1;
%                 row(idx) = (ii-1)*6+l+1;
%                 idx = idx + 1;
%             end
%         end
%     end
    hgx = zeros(6*nelem+1,6*nelem+1);
    hgx = hgx+sparse(row,col,val);
    hgx = hgx+hgx'-diag(diag(hgx));
end

