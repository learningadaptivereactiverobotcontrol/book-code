function [n,m,b,f,ijk,ibound,xy]=topo(iname)
%
%c.^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^;
%c       topo reads input geo./topo data from a disc-file lin        .*;
%c and computes the b matrix                                         .*;
%c                                                                   .*;
%c input:   lin,nmax,mmax,ndim                                       .*;
%c output:  n,m,y,ijk,b,f,ibound,TITLE                               .*;
%c.^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^;

% Matlab coding by Michal Kocvara, University of Birmingham, 2010
% kocvara@maths.bham.ac.uk
%
% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

nodeb=(ones(1,1000)); 

ndim=2;
lin = 1;

NAME=iname;
fid=fopen(NAME,'r+');

title=textscan(fid,'%s',1);
n=fscanf(fid,'%g',[1]);
m=fscanf(fid,'%g',[1]);
for  i=1:n
    kkk=fscanf(fid,'%g',1);
    for l=1:ndim  
        y(l,kkk)=fscanf(fid,'%g',1); 
    end; 
end    

for  i=1:m
    kkk=fscanf(fid,'%g',1);
    ijk(1,kkk)=fscanf(fid,'%g',1);
    ijk(2,kkk)=fscanf(fid,'%g',1);
end

%c.^^^^^^^^^^^^^^^^;
%c   computation of compability matrix;
%c.^^^^^^^^^^^^^^^^;
for  i=1:m
    xl=0.0d0;
    for  k=1:ndim;
        xl=xl+(y(k,ijk(1,i))-y(k,ijk(2,i))).^2;
    end
    for  k=1:ndim;
        b(k,i)=(y(k,ijk(2,i))-y(k,ijk(1,i)))./xl;
        b(k+ndim,i)=-b(k,i);
    end
end

%c.^^^^^^^^^^^^^^^^;
%c   modifying load vector;
%c.^^^^^^^^^^^^^^^^;
f=zeros(n,2);
nlc=fscanf(fid,'%g',1);
nload=fscanf(fid,'%g',1);
for  i=1:nload;
    k=fscanf(fid,'%g',1);
    for l=1:ndim
        f(k,l)=fscanf(fid,'%g',1); 
    end; 
end;

%c.^^^^^^^^^^^^^^^^;
%c   modifying compability matrix forml boundary conditions;
%c.^^^^^^^^^^^^^^^^;

ibound=ones(ndim,n);

nbound=fscanf(fid,'%g',1);
for  i=1:nbound
    kkk=fscanf(fid,'%g',1);
    for l=1:ndim
        ibound(l,kkk)=fscanf(fid,'%g',1); 
    end; 
end;
nodeb(i) = kkk;
% 1130  continue;
fclose(fid);

for  i=1:nbound
    for  l=1:m
        if(ijk(1,l)==nodeb(i))
            for  k=1:ndim;
                if(ibound(k,nodeb(i))==0) ;
                    cb(k,l)=0.0d0;
                end
            end
        end
        if(ijk(2,l)==nodeb(i)) ;
            for  k=1:ndim
                if(ibound(k,nodeb(i))==0) ;
                    cb(k+ndim,l)=0.0d0;
                end
            end
        end
    end
end

%c.....changes w.r.t. ipm code;
for  i=1:m
    ijk(4,i) =  2.*ijk(2,i);
    ijk(3,i) =  2.*ijk(2,i)-1;
    ijk(2,i) =  2.*ijk(1,i);
    ijk(1,i) =  2.*ijk(1,i)-1;
end

xy=y';
b=b';
ijk=ijk';
ibound=ibound';

%reshape(f',[18,1]);


