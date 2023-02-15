function par = kobum(iname)
%
%   reads the input data and creates the structure
%   "par" of the problem parameters

% Matlab coding by Michal Kocvara, University of Birmingham, 2010
% kocvara@maths.bham.ac.uk
%
% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

[nhalf,m,b,f,ijk,ibound,xy]=topo(iname);

for i=1:m
  for j=1:2:3
    BI(i,ijk(i,j)  ) = b(i,j  );
    BI(i,ijk(i,j+1)) = b(i,j+1);
  end
end

n = nhalf+nhalf;

ib=reshape(ibound',n,1);
maska=find(ib);

n1 = length(maska);

fhelp=reshape(f',n,1);
ff=fhelp(maska);

par.n = n; par.n1 = n1; par.m = m;
par.maska = maska;
par.f = ff;
par.BI = BI;
%par.DELTA = DELTA;
par.xy = xy;
par.ijk = ijk;

pic_ini(par);
