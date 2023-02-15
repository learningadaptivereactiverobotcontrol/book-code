function [dgx, data] = fmo_congrad(x, Y, data)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

AA = data.AA;
d = data.d;

nelem = length(Y);
dgx = sparse(6*nelem+1,nelem+2);

%first

val = zeros(6*nelem+1,1);
val(1) = -1;
idx = 2;
for ii=1:nelem
    for k=[1 2 4 3 5 6]
        val(idx) = -d'*AA{ii,k}*d;
        idx = idx + 1;
    end
end
dgx(1:6*nelem+1,1) = val;

%second
dgx(1,2) = 0;
v =[1;0;0;1;0;1];
dgx(2:6*nelem+1,2) = kron(ones(nelem,1),v);

%traces
idx = 2;
for i=1:nelem
    dgx(1,2+i) = 0;
    dgx(idx:idx+5,2+i) = v;
    idx = idx+6;
end
end

