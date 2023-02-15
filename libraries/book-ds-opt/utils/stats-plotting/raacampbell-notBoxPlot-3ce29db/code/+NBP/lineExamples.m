function lineExamples

% % Mixing lines and areas and using different number of points
% % in each group [note that the way notBoxPlot sets the x axis 
% % limits can cause problems when combining plots this way]
%
% clf
%
% subplot(1,2,1)
% h=notBoxPlot(randn(10,1)+4,5,'style','line');
% set(h.data,'color','m')
% h=notBoxPlot(randn(50,10));
% set(h(5).data,'color','m')
%
% subplot(1,2,2)
% notBoxPlot(randn(25,1),1,'style','sdline')
% notBoxPlot(randn(50,1),2) % more points
% xlim([0,3])

help(['NBP.',mfilename])

clf

subplot(1,2,1)
h=notBoxPlot(randn(10,1)+4,5,'style','line');
set(h.data,'color','m')  
h=notBoxPlot(randn(50,10));
set(h(5).data,'color','m')

subplot(1,2,2)
notBoxPlot(randn(25,1),1,'style','sdline')
notBoxPlot(randn(50,1),2) % more points
xlim([0,3])
