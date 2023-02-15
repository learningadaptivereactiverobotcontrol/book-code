function [gx, data] = fmo_confun(x, Y, data)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

nnzmax = data.nnzmax;
AA = data.AA;
fff = data.fff;

nnod = length(fff);
nelem = length(Y);

kk=0;
%first
AStiff=spalloc(nnod,nnod,nnzmax);
for ii=1:nelem
    xY = Y{ii}; xx = [xY(1,1);xY(1,2);xY(2,2);xY(1,3);xY(2,3);xY(3,3)];
    for jj=1:6
        AStiff = AStiff+xx(jj)*AA{ii,jj};
    end
end

[R,p] = chol(AStiff);
if p ~= 0
    gx(1,1) = 1.0e38;
    %display(' INFEASIBLE Y');
    kk=1;
%    return;
end

if kk==0
d = R\(R'\fff);
gx(1,1) = fff'*d - x(1);
data.d = d;
data.R = R;
end

%second
gxx = 0;
for i=1:nelem
    gxx = gxx + trace(Y{i});
end
gx(2,1) = gxx;

%traces
for i=1:nelem
    gx(i+2,1) = trace(Y{i});
end
end


