function linearModelExamples

% % Linear model call format
%
%
% % Build data
% rng(555), 
% n=10;
% R=rand(n,5);
% R(:,3)=R(:,3)+1;
% 
% X=repmat(1:5,size(R,1),1);
% lemmings=R(:);
% group=X(:);
%
% clf
%
% % We can call notBoxPlot with just X and Y
% subplot(2,2,1)
% notBoxPlot(lemmings,group,'jitter',0.75)
% grid on, box on
% ylim([-0.5,2.2])
% title('two vectors')
%
% % We can create a table and get the same plot plus the variable names on the axes
% subplot(2,2,2)
% T = table(lemmings,group);
% notBoxPlot(T,'jitter',0.75)
% grid on, box on
% ylim([-0.5,2.2])
% title('table')
%
% % We can fit a linear model do the data and plot this
% subplot(2,2,3)
% group = categorical(group);
% T = table(lemmings,group);
% M = fitlm(T,'lemmings ~ group');
% notBoxPlot(M,'jitter',0.75)
% grid on, box on
% ylim([-0.5,2.2])
% title('model')
%
% % Increase variance of one group
% subplot(2,2,4)
% lemmings(end-n+1:end) = lemmings(end-n+1:end)*1.75;
% T = table(lemmings,group);
% M = fitlm(T,'lemmings ~ group');
% notBoxPlot(M,'jitter',0.75)
% grid on, box on
% ylim([-0.5,2.2])
% title('increased variance in group 5')

help(['NBP.',mfilename])




% Build data
rng(555), 
n=10;
R=rand(n,5);
R(:,3)=R(:,3)+1;

X=repmat(1:5,size(R,1),1);
lemmings=R(:);
group=X(:);


clf

% We can call notBoxPlot with just X and Y
subplot(2,2,1)
notBoxPlot(lemmings,group,'jitter',0.75)
grid on, box on
ylim([-0.5,2.2])
title('two vectors')

% We can create a table and get the same plot plus the variable names on the axes
subplot(2,2,2)
T = table(lemmings,group);
notBoxPlot(T,'jitter',0.75)
grid on, box on
ylim([-0.5,2.2])
title('table')

% We can fit a linear model do the data and plot this
subplot(2,2,3)
group = categorical(group);
T = table(lemmings,group);
M = fitlm(T,'lemmings ~ group');
notBoxPlot(M,'jitter',0.75)
grid on, box on
ylim([-0.5,2.2])
title('model')


% Increase variance of one group
subplot(2,2,4)
lemmings(end-n+1:end) = lemmings(end-n+1:end)*1.75;
T = table(lemmings,group);
M = fitlm(T,'lemmings ~ group');
notBoxPlot(M,'jitter',0.75)
grid on, box on
ylim([-0.5,2.2])
title('increased variance in group 5')
