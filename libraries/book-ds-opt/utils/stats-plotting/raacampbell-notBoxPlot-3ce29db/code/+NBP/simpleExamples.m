function simpleExamples
%
% clf 
%
% subplot(2,2,1)  
% notBoxPlot(randn(20,5));
%
% subplot(2,2,2)  
% notBoxPlot(randn(20,5),[1:4,7]);
%
% subplot(2,2,3)
% h=notBoxPlot(randn(10,20));
% d=[h.data];
% set(d(1:4:end),'markerfacecolor',[0.4,1,0.4],'color',[0,0.4,0])
%
% subplot(2,2,4)
% x=[1,2,3,4,5,5];
% y=randn(20,length(x));
% y(:,end)=y(:,end)+3;
% y(:,end-1)=y(:,end-1)-1;
% notBoxPlot(y,x);

help(['NBP.',mfilename])

clf 

subplot(2,2,1)  
notBoxPlot(randn(20,5));

subplot(2,2,2)  
notBoxPlot(randn(20,5),[1:4,7]);

subplot(2,2,3)
h=notBoxPlot(randn(10,20));
d=[h.data];
set(d(1:4:end),'markerfacecolor',[0.4,1,0.4],'color',[0,0.4,0])


subplot(2,2,4)
x=[1,2,3,4,5,5];
y=randn(20,length(x));
y(:,end)=y(:,end)+3;
y(:,end-1)=y(:,end-1)-1;
notBoxPlot(y,x);
