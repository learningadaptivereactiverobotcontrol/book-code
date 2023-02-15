function [n,m,A0,AA,mtred]=pmisof(fname,scaled)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

if nargin < 2
    scaled = 0;
end

[A,B1,B,C1,C]=COMPleib(fname);

K=sdpvar(size(B,2),size(C,1),'full') ;
H=hermitesof(A,B,C,K) ;
mt = yalmip('monomtable');

n = size(K,1)*size(K,2);    % no of variables ( including lambda)
m = size(A,1);        % size of matrices

mtred = full(mt(getvariables(H),1:n));
%mtred = full(mt(getvariables(H),:));
nnz = size(mtred,1);
nzidx = getvariables(H);

if length(nzidx) ~= nnz
    warning('Inconsistent data!');
end

A0 = getbasematrix(H,0);

if scaled ~= 0
    ee=eig(full(A0));
    if(min(ee) <= 1.0e-7)
        error('Scaling problem!');
    end
    [S,lambda]=eig(full(A0));
    lambdaS=sqrt(inv(lambda));
    diagM=S*lambdaS*S';
    A0 = diagM*A0*diagM;
end

AA = {};

for i=1:nnz
   mtline = mtred(i,:);
   if scaled ~= 0
       AA{i} = diagM*getbasematrix(H,nzidx(i))*diagM;
   else
       AA{i} = getbasematrix(H,nzidx(i));
   end
end



% check for degree higher than 8
dmax  = 0;
for i=1:nnz
   mtline = mtred(i,:);
   if sum(mtline) > dmax
        dmax = sum(mtline);
   end        
end

outp=sprintf('maximal degree %d', dmax);
disp(outp);
outp=sprintf('dimensions %d, %d', n, m);
disp(outp);

if dmax > 8
    fclose(outfile);
    warning('Polynomials of degree higher than 8 detected (degr=%d)\n', dmax);
end


