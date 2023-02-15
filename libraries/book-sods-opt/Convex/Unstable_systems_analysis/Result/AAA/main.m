close all
neworder = {
    'Orginal_{QP}_{-Training}'            [mean(ERROR_CON(1,:))]
    'Orginal_{QP}_{-Testing}'            [mean(ERROR_CON_T(1,:))]
    'Orginal_{EM}_{-Training}'            [mean(ERROR_EM(1,:))]
    'Orginal_{EM}_{-Testing}'            [mean(ERROR_EM_T(1,:))]
    }
figure1 = figure;

axes1 = axes('Parent',figure1,'XTickLabel',neworder(:,1));
box(axes1,'on');
hold(axes1,'all');
ERROR=[ERROR_CON',ERROR_CON_T',ERROR_EM',ERROR_EM_T']
meann = mean(ERROR)
stdd = std(ERROR)


for i=1:size(ERROR,2)
    Y(1,i)=mean(ERROR(:,i));
    lower(1,i)=min(ERROR(:,i));
    upper(1,i)=max(ERROR(:,i));
end


% Y=[mean(ERROR(:,1)),mean(ERROR(:,2)), mean(ERROR(:,3)), mean(ERROR(:,4)), mean(ERROR(:,5)), mean(ERROR(:,6)) , mean(ERROR(:,7))];
% E=[sqrt(var(ERROR(:,1))),sqrt(var(ERROR(:,2))), sqrt(var(ERROR(:,3))), sqrt(var(ERROR(:,4)))];
% lower=[min(ERROR(:,1)),min(ERROR(:,2)), min(ERROR(:,3)), min(ERROR(:,4)), min(ERROR(:,5)), min(ERROR(:,6)), min(ERROR(:,7))];
% upper=[max(ERROR(:,1)),max(ERROR(:,2)), max(ERROR(:,3)), max(ERROR(:,4)), max(ERROR(:,5)), max(ERROR(:,6)), max(ERROR(:,7))];
% Y=[mean(Likelihood(:,1)),mean(Likelihood(:,2)), mean(Likelihood(:,3)), mean(Likelihood(:,4)), mean(Likelihood(:,5))];
% E=[sqrt(var(Likelihood(:,1))),sqrt(var(Likelihood(:,2))), sqrt(var(Likelihood(:,3))), sqrt(var(Likelihood(:,4)))];
% lower=[min(Likelihood(:,1)),min(Likelihood(:,2)), min(Likelihood(:,3)), min(Likelihood(:,4)), min(Likelihood(:,5))];
% upper=[max(Likelihood(:,1)),max(Likelihood(:,2)), max(Likelihood(:,3)), max(Likelihood(:,4)), min(Likelihood(:,5))];
errorbar([1 2 3 4],[Y(:,1),Y(:,2),Y(:,3),Y(:,4)],[lower(:,1),lower(:,2),lower(:,3),lower(:,4)],[upper(:,1),upper(:,2),upper(:,3),upper(:,4)],'rx')
set(gca,'xtick',1:4,'XTickLabel',neworder(:,1))


createfigure((1:25), [time_CON;time_EM])

% Y=[mean(reshape(ERROR(:,1,:),1,12)),mean(reshape(ERROR(:,2,:),1,12)), mean(reshape(ERROR(:,3,:),1,12)),...
%     mean(reshape(ERROR(:,4,:),1,12)),mean(reshape(ERROR(:,5,:),1,12)), mean(reshape(ERROR(:,6,:),1,12)),...
% mean(reshape(ERROR(:,7,:),1,12)),mean(reshape(ERROR(:,8,:),1,12)), mean(reshape(ERROR(:,9,:),1,12)),...
% mean(reshape(ERROR(:,10,:),1,12)),mean(reshape(ERROR(:,11,:),1,12))];
% E=[sqrt(var(reshape(ERROR(:,1,:),1,12))),sqrt(var(reshape(ERROR(:,2,:),1,12))),sqrt(var(reshape(ERROR(:,3,:),1,12))),...
%     sqrt(var(reshape(ERROR(:,4,:),1,12))),sqrt(var(reshape(ERROR(:,5,:),1,12))),sqrt(var(reshape(ERROR(:,6,:),1,12))),...
% sqrt(var(reshape(ERROR(:,7,:),1,12))),sqrt(var(reshape(ERROR(:,8,:),1,12))),sqrt(var(reshape(ERROR(:,9,:),1,12))),...
% sqrt(var(reshape(ERROR(:,10,:),1,12))),sqrt(var(reshape(ERROR(:,11,:),1,12)))];