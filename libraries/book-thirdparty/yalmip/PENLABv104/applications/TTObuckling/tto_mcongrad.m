function [Akdx, userdata] = sdp_mcongrad(x,Y,k,i,userdata)
% Compute derivatives: d/dx_i A_k(x) based on the data from sdpdata
% in this context it is in fact -F_i of the specific block

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

Akdx=[];
if (k<=0 || k>userdata.Na)
    return;
end
if (i<=0 || i>userdata.Nx)
    return;
end

if k==2
    n1 = length(userdata.A{1,1})-1;
    Akdx=[0 sparse(1,n1); sparse(n1,1) userdata.A{1,i+1}];
else
    par = userdata.par;
    m=par.m; n=par.n; n1 = par.n1;
    BI=par.BI; DELTA=par.DELTA; f=par.f;
    
    Ah=sparse(n1,n1);
    for j=1:m
        Ah=Ah+x(j)*userdata.A{1,j+1};
    end
    
    der = userdata.A{1,i+1};
    der = der - BI(i,:)*(Ah\f)*reshape(DELTA(:,i),n1,n1);
    paraf = Ah\(userdata.A{1,i+1}*(Ah\f));
    
    for j=1:m
        der = der + x(j)*(BI(j,:)*paraf*reshape(DELTA(:,j),n1,n1));
    end
    Akdx = sparse(der);
end

