function jitterExamples

% 
% % Jitter examples 
% % default jitter is 0.3
%
% clf
%
% R = randn(40,5);
%
% subplot(3,1,1)
% notBoxPlot(R,'jitter',0.15)
%
% subplot(3,1,2)
% notBoxPlot(R,'jitter',0.3); % The default
%
% subplot(3,1,3)
% notBoxPlot(R,'jitter',0.6);


help(['NBP.',mfilename])

clf

R = randn(40,5);

subplot(3,1,1)
notBoxPlot(R,'jitter',0.15)

subplot(3,1,2)
notBoxPlot(R,'jitter',0.3); % The default

subplot(3,1,3)
notBoxPlot(R,'jitter',0.6);
