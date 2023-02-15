function showCase
% Example showing a variety of effects possible with notBoxPlot
%
% function NBP.showCase
%
% 
% Purpose
% Showcase notBoxPlot
%
%
% No inputs or outputs
%
% 
% Rob Campbell



W=which(['NBP.',mfilename]);
fprintf('Running example located at %s\n',W)

hFig=figure(1984);

set(hFig,...
    'Name','notBoxPlot example',...
    'PaperPosition',[0,0,32,27]) %Just to make save to disk consistent)
clf

W=0.45; %image width

% Top/left plot
axes('position',[0.05,0.53,W,W])
r=randn(40,5);
for ii=1:5
  r(:,ii)=r(:,ii)+ii*0.3;
end
notBoxPlot(r,[],'jitter',0.5);
box on
grid on



% Top/right plot
axes('position',[0.53,0.53,W,W])
r=randn(20,20);

IND=zeros(1,20);
IND(1:4:size(r,2))=1;
r(:,find(IND))=0.75*r(:,1:4:end)+1.75;

H=notBoxPlot(r,[],'jitter',0.6);
d=[H.data];

%Highlight the plots with higher means
set(d(find(IND)),'markerfacecolor',[0.4,1,0.4],'color',[0,0.4,0])

%higher means as green
set([H(find(IND)).data],'MarkerSize',4,...
    'markerFaceColor',[1,1,1]*0.25,...
    'markerEdgeColor', 'none')
set([H(find(IND)).semPtch],...
    'FaceColor',[0,0.75,0],...
    'EdgeColor','none')
set([H(find(IND)).sdPtch],...
    'FaceColor',[0.6,1,0.6],...
    'EdgeColor','none')
set([H(find(IND)).mu],...
    'Color',[0,0.4,0])

set(gca,'XTick',[]) 


% Color lower means gray
set([H(find(~IND)).data],'MarkerSize',4,...
    'markerFaceColor',[1,1,1]*0.5,...
    'markerEdgeColor', 'none')
set([H(find(~IND)).semPtch],...
    'FaceColor',[1,1,1]*0.25,...
    'EdgeColor','none')
set([H(find(~IND)).sdPtch],...
    'FaceColor',[1,1,1]*0.75,...
    'EdgeColor','none')
set([H(find(~IND)).mu],...
    'Color','b')

box on



axes('position',[0.05,0.05,W,W])
x=[1,2,3,3];
y=randn(20,length(x));
y(:,end)=0.5*y(:,end)+4;
y(:,end-1)=y(:,end-1)-1;
y(1:8,end-1:end)=nan; %Decrease sample size in the last two plots

H=notBoxPlot(y,x,'jitter',0.6,'style','sdline');
set(H(end).data,'Marker','^',...
    'MarkerSize',5)
set([H.sd],'LineWidth',4)
box on
grid on



axes('position',[0.53,0.05,W,W])
H=notBoxPlot(randn(10,1)+7,2,'style','line');
set(H.data,'color','b','Marker','.') 

r=randn(20,10);
for ii=1:10
  r(:,ii)=r(:,ii)+ii*0.65;
end

H=notBoxPlot(r,[],'jitter',0.5);
set([H.data],...
    'MarkerFaceColor',[1,1,1]*0.35,...
    'markerEdgeColor',[1,1,1]*0.35,...
    'MarkerSize',3)

set([H.mu],'color','w')
J=jet(length(H));
for ii=1:length(H)
  set(H(ii).sdPtch,'FaceColor',J(ii,:),...
                   'EdgeColor','none')

  set(H(ii).semPtch,'FaceColor',J(ii,:)*0.3,...
                   'EdgeColor','none')
  
end
box on
set(gca,'TickDir','Out')

