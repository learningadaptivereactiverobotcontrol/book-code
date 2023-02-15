function statsOptionsExamples

% 
% % Examples of different statistics options
% 
% % The 95% SEM vs the 95% t-interval
% clf
% y=randn(8,3);
% subplot(2,2,1)
% notBoxPlot(y)
% title('95% SEM (n=8)')
%
% subplot(2,2,2)
% notBoxPlot(y,'interval','tInterval')
% title('95% t-interval (n=8)')
%
% % Adding medians
% subplot(2,2,:3:4)
% n=[5,10,20,40];
% for ii=1:4
%  rng(555), notBoxPlot(rand(1,n(ii)),ii,'markMedian',true)
% end
% title('median vs mean')


help(['NBP.',mfilename])

% The 95% SEM vs the 95% t-interval
clf
y=randn(8,3);
subplot(2,2,1)
notBoxPlot(y)
title('95% SEM (n=8)')

subplot(2,2,2)
notBoxPlot(y,'interval','tInterval')
title('95% t-interval (n=8)')

% Adding medians
subplot(2,2,3:4)
n=[5,10,20,40];
for ii=1:4
 rng(555), notBoxPlot(rand(1,n(ii)),ii,'markMedian',true)
end
title('median vs mean')