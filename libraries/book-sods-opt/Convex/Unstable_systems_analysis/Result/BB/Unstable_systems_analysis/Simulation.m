function Simulation(Input_S,Unstable,Stable,flag,i)
%% Simulation
Training_N=[3,4,5,7,8,9,11,12,16,17,20];
D=[-15 5 -10 10];
ax.XLim = D(1:2);
ax.YLim = D(3:4);
if flag(1)==1
    %     figure2 = figure;
    h=subplot(4,2,2*i-1,'FontSize',1);
%         plotGMM(Unstable.Mu(1:2,:),Unstable.Sigma(1:2,1:2,:),[0.1 0.5 0.7],1)
%         hold on
    for j=1:size(Training_N,2)
        plot(Input_S{1,Training_N(1,j)}(:,1),Input_S{1,Training_N(1,j)}(:,2),'LineWidth',1,'Color',[1 0 0])
        hold on
    end
    box(h,'on');
    grid(h,'on');
    hold(h,'on');
    ylabel('\xi(1,2)','FontSize',20);
    xlabel('\xi(1,1)','FontSize',20);

    Simulate_stream_line(Unstable.prior,Unstable.Mu,Unstable.Sigma,Unstable.A,D);
    hold on
    plot(0,0,'DisplayName','The target',...
        'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
        'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
        'MarkerSize',20,...
        'Marker','hexagram',...
        'LineStyle','none');
    axis([ax.XLim ax.YLim]);box on
end
if flag(2)==1
    %     figure3 = figure;
    h=subplot(4,2,2*i,'FontSize',1);
%         plotGMM(Stable.Mu(1:2,:),Stable.Sigma(1:2,1:2,:),[0.1 0.5 0.7],1)
%         hold on
    for j=1:size(Training_N,2)
        plot(Input_S{1,Training_N(1,j)}(:,1),Input_S{1,Training_N(1,j)}(:,2),'LineWidth',1,'Color',[1 0 0])
        hold on
    end
    box(h,'on');
    grid(h,'on');
    hold(h,'on');
    ylabel('\xi(1,2)','FontSize',20);
    xlabel('\xi(1,1)','FontSize',20);

    Simulate_stream_line(Stable.prior,Stable.Mu,Stable.Sigma,Stable.A,D);
    hold on
    plot(0,0,'DisplayName','The target',...
        'MarkerFaceColor',[0 0.447058826684952 0.74117648601532],...
        'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
        'MarkerSize',20,...
        'Marker','hexagram',...
        'LineStyle','none');
    axis([ax.XLim ax.YLim]);box on
end