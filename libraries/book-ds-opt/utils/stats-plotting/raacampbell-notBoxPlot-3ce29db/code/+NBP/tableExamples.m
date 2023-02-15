function tableExamples

% % Table call format
%
% clf
%
% albert=[1,1,1,3,2,1,3,3,3,2,2,3,3]';
% victoria=[7,8,6,1,5,7,2,1,3,4,5,2,4]';
% M = table(victoria,albert); %place data in first column and groups in the second
%
% subplot(1,2,1)
% notBoxPlot(M)
%
% subplot(1,2,2)
% notBoxPlot(M,'jitter',0.5)


help(['NBP.',mfilename])


clf

albert=[1,1,1,3,2,1,3,3,3,2,2,3,3]';
victoria=[7,8,6,1,5,7,2,1,3,4,5,2,4]';

M = table(victoria,albert); %place data in first column and groups in the second

subplot(1,2,1)
notBoxPlot(M)

subplot(1,2,2)
notBoxPlot(M,'jitter',0.75)