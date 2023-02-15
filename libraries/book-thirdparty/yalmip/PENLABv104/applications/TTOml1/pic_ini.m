function pic_ini(par)

%   plots the initial groundstructure,
%
% Matlab coding by Michal Kocvara, University of Birmingham, 2010
% kocvara@maths.bham.ac.uk
%

xy = par.xy; m=par.m; ijk = par.ijk;
n=par.n;
maska=par.maska; BI=par.BI; f=par.f;

t = ones(m,1);

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
tscale=50/tmax/sqrt(m);

clf

for i=1:m
   if(abs(vol(i)) > 0.001)
      th =tscale*vol(i);
      th=2;th=.5;

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

% 3x3ff single
 nloads = 1;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(9) =  10;
% nloads = 2;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(7) =  10;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(11) = 10;
% 5x5ff
% nloads = 3;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(31) = 10; par.ff{1}(32) =  -10;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(39) = 10;
% par.ff{3} = zeros(par.n1,1); par.ff{3}(26) = 10;

par.ff{1} = zeros(par.n1,1); %par.ff{1}(31) =  10;
 par.ff{1}(40) =  -10;

xym = xy(maska(2:2:end)/2,:);
for i=1:nloads
    fff = par.ff{i};
    I = find(fff);
    nonu = ceil(I(1)/2);
    I = [nonu*2-1;nonu*2];
    
    for j=1:length(I)/2
        x1(i) = xym(I(j+1)/2,1); x2(i)=xym(I(j+1)/2,2);
        y1(i) = fff(I(j))/(10*norm(fff)); y2(i) = fff(I(j+1))/(10*norm(fff));
    end
end
%quiver(x1+.03,x2,y1,y2,0.35,'-k','LineWidth',5,'MaxHeadSize',2.5); axis auto; pause(0.1);
quiver(x1,x2,y1,y2,0.35,'-k','LineWidth',5,'MaxHeadSize',2.5); axis auto; pause(0.1);

bcon=setdiff(1:n,maska);
bcon=bcon(2:2:end)./2;
for i=1:length(bcon)
    x = xy(bcon(i),1);y = xy(bcon(i),2);
    plot(x,y,'ks','MarkerEdgeColor','k',...
                       'MarkerFaceColor','k','MarkerSize',25);
end

axis('equal');
axis('off');
titi=0.05;
axis([xmin-titi xmax+titi+.2 ymin-titi-.2 ymax+titi]);

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

set(gcf, 'renderer', 'painters');
