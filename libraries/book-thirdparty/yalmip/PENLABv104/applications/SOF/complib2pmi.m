function pmi = complib2pmi(fname)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

save fname fname
clear all
load fname

[n,m,A0,AA,mtred]=pmisof2(fname,0);

max_degree = max(sum(mtred'));
matrices = size(AA,2);

midx = zeros(max_degree,matrices+2);
for i=1:matrices
    kk=1;
    for j=1:n
        for k=1:mtred(i,j)
            midx(kk,i+1) = j;
            kk=kk+1;
        end
    end
end

midx(1,matrices+2) = n+1; 

pmi.name = fname;
pmi.Nx = n + 1;
pmi.Na = 1;
pmi.c = [zeros(n,1);-1];
pmi.lbx = -Inf.*ones(n+1,1);
pmi.ubx = Inf.*ones(n+1,1);
%pmi.lbx = [-100.*ones(n,1);-Inf];
%pmi.ubx = [100.*ones(n,1);Inf];

Q=cell(matrices+2,1);
Q{1} = A0;
for i=1:matrices
    Q{i+1} = AA{i};
end
Q{matrices+2} = -eye(m,m);

pmi.A{1}.midx = midx;
pmi.A{1}.Q = Q;

pmi.H = 2.*[eye(n,n) zeros(n,1); zeros(1,n) 0];
pmi.xinit = [rand(n,1);0];
pmi.xinit = [1.*ones(n,1);0];
