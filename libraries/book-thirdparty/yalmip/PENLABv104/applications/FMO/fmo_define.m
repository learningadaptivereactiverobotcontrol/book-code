function [penm] = fmo_define(par)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

Aik=par.Aik;
RHS=par.RHS;
nelem=par.nelem;
nnod=par.nnod;
nloads=par.nloads;

V = .5*nelem;
mlin = 0;
AA = Aik;
Vi = 1;
rScale=0.1;

AStiff=spalloc(nnod,nnod,10);

% Assembling global stiffness matrix
xx = [1 0 1 0 0 1];%;
for ii=1:nelem
    for jj=1:6
        AStiff = AStiff+xx(jj)*AA{ii,jj};
    end
end
nnzmax = nnz(AStiff);
fmodata.AStiff = AStiff;
fmodata.nnzmax = nnzmax;

d = {};
for i=1:1
    d{i} = zeros(nnod,1);
end

fmodata.d=d;
fmodata.V=V;
fmodata.AA=AA;


% Algorithmic parameters
fmodata.rScale = .1;
fmodata.Vi = Inf;


Y0 = 1.1.*eye(3,3);
x0 = 0.;

% Lower & Upper bounds on eigenvalues
El = 3.333e-3;El = 0;
Eu = Vi;

fff = rScale*RHS(:,i);
fmodata.fff = fff;

penm = [];
penm.userdata = fmodata;
penm.Nx = 1;
penm.NY = nelem;
for i=1:nelem
    penm.Y{i} = ones(3,3);
end
penm.lbY = El*ones(nelem,1);
penm.lbYbar = [1:nelem];

penm.NgNLN = 1;
penm.NgLIN = 1+nelem;
penm.ubg = [0;V;Eu*ones(nelem,1)];

penm.xinit=x0;
for i=1:nelem
    penm.Yinit{i}=Y0;
end

penm.objfun = @fmo_objfun;
penm.objgrad = @fmo_objgrad;
penm.objhess = @fmo_objhess;

penm.confun = @fmo_confun;
penm.congrad = @fmo_congrad;
penm.conhess = @fmo_conhess;
