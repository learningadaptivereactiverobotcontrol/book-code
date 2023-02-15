function [Akddx, userdata] = sdp_mconhess(x,Y,k,i,j,userdata)
% Compute 2nd derivatives: d/dx_i A_k(x) based on the data from sdpdata
% in this context it is in fact -F_i of the specific block

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

Akddx=[];
if (k<=0 || k>userdata.Na)
    return;
end
if (i<=0 || i>userdata.Nx)
    return;
end

if k==2
    Akddx=[];
else
    par = userdata.par;
    m=par.m; n=par.n; n1 = par.n1;
    BI=par.BI; DELTA=par.DELTA; f=par.f;
    
    if i==1 & j==1
        Ah=sparse(n1,n1);
        for ii=1:m
            Ah=Ah+x(ii)*userdata.A{1,ii+1};
        end
        userdata.Ah = Ah;
    else
        Ah = userdata.Ah;
    end
    
    der = sparse(n1,n1);
    
    parafi = Ah\(userdata.A{1,i+1}*(Ah\f));
    parafj = Ah\(userdata.A{1,j+1}*(Ah\f));
    der = der + BI(j,:)*parafi*reshape(DELTA(:,j),n1,n1);
    der = der + BI(i,:)*parafj*reshape(DELTA(:,i),n1,n1);
    
    parafij = Ah\(userdata.A{1,i+1}*(Ah\(userdata.A{1,j+1}*(Ah\f))));
    parafji = Ah\(userdata.A{1,j+1}*(Ah\(userdata.A{1,i+1}*(Ah\f))));
    parafijji = parafij+parafji;
    BIx = (BI.*repmat(x,[1,n1]))*parafijji;
    der = der - reshape(DELTA*BIx,n1,n1);
%     for jj=1:m
%         der = der - ((BIx(jj)).*reshape(DELTA(:,jj),n1,n1));
%  %      der = der - ((x(jj).*(BI(jj,:))*parafijji).*DELTA{jj});
%  %      der = der - x(jj)*(BI(jj,maska)*parafij*DELTA{jj}+BI(jj,maska)*parafji*DELTA{jj});
%     end
     
    Akddx = der;
end

