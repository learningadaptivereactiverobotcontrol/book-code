function [Akx, userdata] = sdp_mconfun(x,Y,k,userdata)
% evaluate A_k(x) based on sdpdata, k denotes a block number
% sdpdata is a structure as obtained from readsdpa.m
% note that we aim for A_k(x)<=0 thus A_k(x) is 'reversed' than usual:
%   A_k(x) = F_0 - sum x_i*F_i

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

Akx=[];
if (k<=0 || k>userdata.Na)
    return;
end

if k==2
    Akx=userdata.A{1,1};
    n1 = length(Akx)-1;
    for i=1:userdata.Nx
        Akx = Akx + x(i).*[0 sparse(1,n1); sparse(n1,1) userdata.A{1,i+1}];
    end
else
    par = userdata.par;
    m=par.m; n=par.n; n1 = par.n1;
    BI=par.BI; DELTA=par.DELTA; f=par.f;
    
    K=sparse(n1,n1);
    for i=1:m
        K=K+x(i)*userdata.A{1,i+1};
    end
    
    ainvf = K\f;
    
    for i=1:m
        K = K - x(i)*BI(i,:)*ainvf*reshape(DELTA(:,i),n1,n1);
    end
    Akx = sparse(K);
end

