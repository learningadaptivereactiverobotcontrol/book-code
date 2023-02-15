function [demos, limits] = load_LASA_dataset()
% This is a matlab function illustrating 30 human handwriting motions
% recorded from Tablet-PC. These datas can be found in the folder
% 'DataSet'. 
%
% Please acknowledge the authors in any academic publications
% that have made use of this library by citing the following paper:
%
%  S. M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear Dynamical 
%  Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011.
%
% To get latest upadate of the software please visit
%                          http://lasa.epfl.ch/khansari
%
% Please send your feedbacks or questions to:
%                           mohammad.khansari_at_epfl.ch

%% Modified by NADIA FIGUEROA. Sept 2018.
%%
names = {'Angle','BendedLine','CShape','DoubleBendedLine','GShape',...
         'heee','JShape','JShape_2','Khamesh','Leaf_1',...
         'Leaf_2','Line','LShape','NShape','PShape',...
         'RShape','Saeghe','Sharpc','Sine','Snake',...
         'Spoon','Sshape','Trapezoid','Worm','WShape','Zshape',...
         'Multi_Models_1', 'Multi_Models_2', 'Multi_Models_3','Multi_Models_4'};

figName = [];
n = -1;
% while true
c = 0;
    fprintf('\nAvailable Models:\n')
    for i=1:8
        for j=1:5
            c=c+1;
            if c > 37
                break;
            end
            fprintf('%2u) %-18s',(i-1)*4+j,names{(i-1)*4+j})
        end
        fprintf('\n')
    end

    n = input('\nType the number of the model you wish to plot [type 0 to exit]: ');
    fprintf('\n\n')
    if n<0 || n>30
        disp('Wrong model number!')
        disp('Please try again and type a number between 1-30.')
    elseif n == 0
        return
    else
        %% preprocessing
        load(['DataSet/' names{n}],'demos','dt') %loading the model
        % Global Attractor of DS
        att_g = [0 0]';

        sample = 2;
        Data = []; x0_all = [];
        for l=1:3   
            % Check where demos end and shift    
            data_ = [demos{l}.pos(:,1:sample:end); demos{l}.vel(:,1:sample:end);];    
            Data = [Data data_];
            x0_all = [x0_all data_(1:2,20)];
            clear data_
        end
        
        
%         % plotting the result
%         if isempty(figName) || ~ishandle(figName)
%             figName = figure('name','Results from Simulation','position',[100   100   600   800]);
%         end
%         sp(1) = subplot(2,1,1);
%         hold on; box on; grid on
%         sp(2) = subplot(2,1,2);
%         hold on; box on; grid on
%         for i=1:length(demos)
%             lg(3) = plot(sp(1),demos{i}.pos(1,:),demos{i}.pos(2,:),'r.');
%             lg(2) = plot(sp(1),demos{i}.pos(1,1),demos{i}.pos(2,1),'ok','markersize',5,'linewidth',5);
% 
%             plot(sp(2),demos{i}.t,sqrt(sum(demos{i}.vel.^2,1)),'r.');
%         %     plot(sp(2),0,sqrt(sum(demos{i}.vel(:,1).^2)),'ok','markersize',5,'linewidth',5);
%         end
%         lg(1) = plot(sp(1),0,0,'k*','markersize',15,'linewidth',3);
% 
%         xlabel(sp(1),'x (mm)','fontsize',15);
%         ylabel(sp(1),'y (mm)','fontsize',15);
%         title(sp(1),names{n},'fontsize',15)
%         axis(sp(1),'tight')
%         ax=get(sp(1));
%         limits = [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
%               ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10];
%         axis(sp(1),limits);
% 
%         xlabel(sp(2),'time (sec)','fontsize',15);
%         ylabel(sp(2),'speed (mm/s)','fontsize',15);
%         title(sp(2),'speed profile','fontsize',15)
%         axis(sp(2),'tight')
%         ax=get(sp(2));
%         axis(sp(2),[ax.XLim ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
%         %%
%         set(sp(1),'position',[0.1300    0.55    0.7750    0.35])
%         set(sp(2),'position',[0.1300    0.08    0.7750    0.35])
%         l = legend(lg,'Target','Starting Points','Demonstrations','orientation','horizontal');
%         set(l,'position',[0.2133    0.9617    0.6272    0.0300],'fontsize',10)
%     end
end