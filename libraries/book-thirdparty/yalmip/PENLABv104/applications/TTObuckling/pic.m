function pic(par,t)

%   Prints some output values,
%   stores the "t" in the file t.dat
%   and plots the optimal structure

% Matlab coding by Michal Kocvara, University of Birmingham, 2010
% kocvara@maths.bham.ac.uk
%
% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

xy = par.xy; m=par.m; ijk = par.ijk;
n=par.n;
maska=par.maska; BI=par.BI; f=par.f;


ngrap=max(ijk(1:m,:))/2;
ngra=max(ngrap);
vol=zeros(m,1);
for i=1:m
   if(abs(t(i)) > 0.001)
   x1=xy(ijk(i,2)/2,1); y1=xy(ijk(i,2)/2,2);
   x2=xy(ijk(i,4)/2,1); y2=xy(ijk(i,4)/2,2);
   len=sqrt((x1-x2)^2 + (y1-y2)^2);
   %vol(i) = t(i)/len;
   vol(i)=t(i);
   end
end

xmin=min(xy(1:ngra,1));
ymin=min(xy(1:ngra,2));
xmax=max(xy(1:ngra,1));
ymax=max(xy(1:ngra,2));
xymax=max(xmax,ymax);

tmax=max(vol(1:m));
tscale=10/tmax;

clf

for i=1:m
   if(abs(vol(i)) > 0.001)
      th =tscale*vol(i);

      xgra(1)=xy(ijk(i,2)/2,1);
      ygra(1)=xy(ijk(i,2)/2,2);

      xgra(2)=xy(ijk(i,4)/2,1);
      ygra(2)=xy(ijk(i,4)/2,2);
      
      if th > 0
          plot(xgra,ygra,'k-','LineWidth',th);
      else
          plot(xgra,ygra,'k:','LineWidth',-th);          
      end
      hold on
      
   end
end

axis('equal');
axis('off');
axis([xmin-0.1 xmax+0.1 ymin-0.1 ymax+0.1]);


