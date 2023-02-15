function [val] = hvg(i, x, v)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

global d nnnod nnzmax AA nlc R htime fff

nelem6 = length(x)-1;
val = zeros(1,1+nelem6);

if i < nlc
    nnod = length(fff{1});
    Atmp=spalloc(nnod,nnod,nnzmax);  
    %Assembling global stiffness matrix
    idx = 2;
    for ii=1:nelem6/6
        for k=1:6
            Atmp = Atmp+v(idx)*AA{ii,k};
            idx = idx + 1;
        end
    end
        
    dtmp = R\(R'\(Atmp*d{i+1}));
    
    idx = 2;
    for ii=1:nelem6/6
        for k=1:6
            val(idx) = 2*d{i+1}'*AA{ii,k}*dtmp;
            idx = idx + 1;
        end
    end
end

