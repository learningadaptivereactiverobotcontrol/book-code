function varargout=notBoxPlot(y,x,varargin)
% notBoxPlot - Doesn't plot box plots!
%
% function notBoxPlot(y,x,'Param1',val1,'Param2',val2,...)
%
%
% Purpose
% An alternative to a box plot, where the focus is on showing raw
% data. Plots columns of y as different groups located at points
% along the x axis defined by the optional vector x. Points are
% layed over a 1.96 SEM (95% confidence interval) in red and a 1 SD
% in blue. The user has the option of plotting the SEM and SD as a
% line rather than area. Raw data are jittered along x for clarity. This
% function is suited to displaying data which are normally distributed.
% Since, for instance, the SEM is meaningless if the data are bimodally
% distributed. 
%
%
% Inputs
% y - A vector, matrix, or table of the data to plot. 
%      * vector and no x is provided: all data are grouped at one x position.
%      * matrix and no x is provided: each column is plotted in a different x position. 
%      * vector with x grouping variable provided: data grouped according to x
%      * a Table is treated such that the first column is y and the second x.
%      * a LinearModel produced by fitlm
%
% x - [optional], the x axis points at which y columns should be
%     plotted. This allows more than one set of y values to appear
%     at one x location. Such instances are coloured differently. 
%     Contrast the first two panels in Example 1 to see how this input behaves.
%     x need not be provided if y is a table.
%
% Note that if x and y are both vectors of the same length this function
% behaves like boxplot (see Example 5).
%
%
% Parameter/Value pairs
% 'jitter' - how much to jitter the data for visualization
%          (optional). The width of the boxes are automatically
%          scaled to the jitter magnitude. If jitter is empty or
%          missing then a default value of 0.3 is used. 
%
% 'style' - a string defining plot style of the data.
%        'patch' [default] - plots 95% SEM (by default, see below) and SD as a 
%                box using patch objects. 
%        'line' - create a plot where the SD and 95% SEM (see below) are
%                constructed from lines. 
%        'sdline' - a hybrid of the above, in which only the SD is 
%                replaced with a line.
%
% 'interval' - 'SEM' [default] Plots a 95% confidence interval for the mean
%            - 'tInterval' Plots a 95% t-interval for the mean
%            - If a LinearModel from fitlm is provided, interval is always
%              the tInterval and the confidence interval comes from the model.
%
% 'markMedian' - false [default] if true the median value is highlighted
%                The median is highlighted as a dotted line or an open square 
%                (if "line" style was used).
%
%
% Outputs (all area optional)
% H - structure of handles for plot objects.
% stats - the values of the mean, SD, etc, used for the plots
% 
%
% 
% -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - 
% Examples (run clf between examples):
% 
% 1 - Basic usage:
%  >> notBoxPlot([7,8,6,1,5,7,2,1,3,4,5,2,4])
%  >> notBoxPlot([7,8,6,1,5,7,2,1,3,4,5,2,4], [1,1,1,3,2,1,3,3,3,2,2,3,3])
%  >> notBoxPlot(rand(1,100))
%  >> notBoxPlot(randn(20,5))
%  >> notBoxPlot(randn(20,5),[1:4,7]);
%  >> notBoxPlot(MY_TABLE)
%
%  For more run:
%   NBP.simpleExamples
%   NBP.tableExamples
% 
% 2 - Changing plot style
%  >> notBoxPlot(randn(20,5),[],'interval','tinterval'); 
%  >> notBoxPlot(randn(20,5),'style','line'); %also valid: no need for x
%  >> notBoxPlot(MY_TABLE,'jitter',0.5)
%
%  For more run:
%   NBP.lineExamples
%   NBP.jitterExamples
%   NBP.showCase
%
% 3 - Showing different statistics
%  >> notBoxPlot(randn(8,3),'interval','tInterval')
%  >> notBoxPlot(randn(8,3),'markMedian',true)
%
%  For more run:
%   NBP.statsOptionsExamples
%
% 4 - Overlaying different notBoxPlots on one axis
% >> clf
% >> hold on
% >> for ii=1:8; notBoxPlot(rand(1,ii*10),ii), end 
%
%
% Rob Campbell - August 2016
%
% Also see: boxplot




% Check input arguments
if nargin==0
    help(mfilename)
    return
end

% Check if Y is of a suitable class 
if ~isnumeric(y) && ~istable(y) && ~isa(y,'LinearModel')
    fprintf('Variable y is a %s. This is not an allowed input type. see help %s\n',...
        class(y), mfilename)
    return
end

% Parse the different call types
modelCIs=[]; 
tableOrModelCall=false;

switch lower(class(y))

case 'table'
    tableOrModelCall=true;
    if nargin>1 %so user doesn't need to specify a blank variable for x
        if ~isempty(x)
            varargin=[x,varargin];
        end
    end
    thisTable=y;
    varNames=thisTable.Properties.VariableNames;
    if length(varNames) ~= 2
        fprintf('% s can only handle tables with two variables\n',mfilename)
        return
    end
    y = thisTable.(varNames{1});
    x = thisTable.(varNames{2});

case 'linearmodel'
    tableOrModelCall=true;
    if nargin>1 %so user doesn't need to specify a blank variable for x
        if ~isempty(x)
            varargin=[x,varargin];
        end
    end

    thisModel=y;

    if length(thisModel.PredictorNames) >1
        fprintf('% s can only handle linear models with one predictor\n',mfilename)
        return
    end
    y = thisModel.Variables.(thisModel.ResponseName);
    x = thisModel.Variables.(thisModel.PredictorNames{1});

    %Check that x is of a suitable type
    if isnumeric(x)
        fprintf('The model predictor variable should not be continuous\n')
        return
    end
    if iscell(x)
        fprintf('Coercing predictor variable from a cell array to a categorical variable\n')
        x=categorical(x);
    end

    varNames = {thisModel.ResponseName,thisModel.PredictorNames{1}}; %for the axis labels

   % Set the SD bar to have 1.96 standard deviations
    varargin = [varargin,'numSDs',1.96];

    % Get the the confidence intervals from the model
    modelCIs = coefCI(thisModel,0.05);

otherwise %Otherwise Y is a vector or a matrix

    if isvector(y)
        y=y(:); 
    end

    % Handle case where user doesn't supply X, but there are user-supplied param/val pairs. e.g.
    % notBoxPlot(rand(20,5),'jitter',0.5)
    if nargin>2 && ischar(x)
        varargin=[x,varargin];
        x=[];
    end

    % Generate an monotonically increasing X variable if the user didn't supply anything
    % for the grouping variable
    if nargin<2 || isempty(x)
        x=1:size(y,2);
    end

end %switch class(y)


%If x is logical then the function fails. So let's make sure it's a double
x=double(x);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parse input arguments
params = inputParser;
params.CaseSensitive = false;

%User-visible options
params.addParameter('jitter', 0.3, @(x) isnumeric(x) && isscalar(x));
params.addParameter('style','patch', @(x) ischar(x) && any(strncmpi(x,{'patch','line','sdline'},4)) );
params.addParameter('interval','SEM', @(x) ischar(x) && any(strncmpi(x,{'sem','tinterval'},4)) );
params.addParameter('markMedian', false, @(x) islogical(x));

%Options hidden from the user
params.addParameter('numSDs',1, @(x) isnumeric(x) && isscalar(x) && x>=0) 
params.addParameter('manualCI',[], @(x) (isnumeric(x) && isscalar(x)) || isempty(x) )

params.parse(varargin{:});

%Extract values from the inputParser
jitter     = params.Results.jitter;
style      = params.Results.style;
interval   = params.Results.interval;
markMedian = params.Results.markMedian;

%The multiplier for the SD patch. e.g. for 1.96 SDs this value should be 1.96
numSDs = params.Results.numSDs;
manualCI = params.Results.manualCI; %Is used by the recursive call to over-ride the CI when y is a LinearModel

%Set interval function
switch lower(interval)
    case 'sem'
        intervalFun = @NBP.SEM_calc;
    case 'tinterval'
        intervalFun = @NBP.tInterval_calc;
    otherwise
        error('Interval %s is unknown',interval)
end

if jitter==0 && strcmp(style,'patch') 
    warning('A zero value for jitter means no patch object visible')
end


% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
% We now loop through the unique x values, plotting each notBox in turn
% using recursive calls to notBoxPlot.
if isvector(y) && isvector(x) && length(x)>1
    x=x(:);

    if length(x)~=length(y)
        error('length(x) should equal length(y)')
    end

    u=unique(x);
    for ii=1:length(u)
        f = find(x==u(ii));

        %If a model was used, we use the 95% t-intervals it produces
        if ~isempty(modelCIs)
            thisCI = range(modelCIs(ii,:))/2; %the interval is symmetric and we need just this. 
        else
            thisCI =[];
        end

        h(ii)=notBoxPlot(y(f),u(ii),varargin{:},'manualCI',thisCI); %recursive call
    end


    %Make plot look pretty
    if length(u)>1
        xlim([min(u)-1,max(u)+1])
        set(gca,'XTick',u)
    end

    if nargout==1
        varargout{1}=h;
    end

    %If we had a table we can label the axes
    if tableOrModelCall
        ylabel(varNames{1})
        xlabel(varNames{2})
    end

    return % User's call to notBoxPlot never goes beyond here
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


if length(x) ~= size(y,2)
    error('length of x doesn''t match the number of columns in y')
end




% We're going to render points with the same x value in different
% colors so we loop through all unique x values and do the plotting
% with nested functions. Avoid clearing the axes in order to give
% the user more flexibility in combining plot elements.
hold on
[uX,a,b]=unique(x);

H=[];
stats=[];
for ii=1:length(uX)
    f=b==ii;
    [hTemp,statsTemp]=myPlotter(x(f),y(:,f));
    H = [H,hTemp];
    stats = [stats,statsTemp];
end

hold off

%Tidy up plot: make it look pretty
if length(x)>1
    set(gca,'XTick',unique(x))
    xlim([min(x)-1,max(x)+1])
end


%handle the output arguments
if nargout>0
    varargout{1}=H;
end

if nargout>1
    varargout{2}=stats;
end




%Nested functions follow


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [h,statsOut]=myPlotter(X,Y)
    %This is a nested function that shares the caller's namespace

    if isempty(manualCI)
        SEM=intervalFun(Y); %A function handle to a supplied external function
    else
        SEM=manualCI;
    end

    % NaNs do not contribute to the sample size
    if ~any(isnan(Y(:)))
        % So we definitely have no problems with older MATLAB releases or non-stats toolbox installs
        SD=std(Y)*numSDs;
        mu=mean(Y);
        if markMedian
           med = median(Y);
        end
    elseif ~verLessThan('matlab','9.0') %from this version onwards we use the omitnan flag
        SD=std(Y,'omitnan')*numSDs;
        mu=mean(Y,'omitnan');
        if markMedian
           med = median(Y,'omitnan');
        end
    elseif which('nanmean') %Otherwise proceed if stats toolbox is there
        SD=nanstd(Y)*numSDs; 
        mu=nanmean(Y);
        if markMedian
           med = nanmedian(Y);
        end
    else %raise error
        error('You have NaNs in your data set but are running older than R2016a or you have no stats toolbox.')
    end

    %The plot colors to use for multiple sets of points on the same x
    %location
    cols=hsv(length(X)+1)*0.5;
    cols(1,:)=0;
    jitScale=jitter*0.55; %To scale the patch by the width of the jitter

    for k=1:length(X)

        thisY=Y(:,k);
        thisY=thisY(~isnan(thisY));
        thisX=repmat(X(k),1,length(thisY));

        %Assemble stats for optional command line output
        statsOut(k).mu = mu(k);
        statsOut(k).interval = SEM(k);
        statsOut(k).sd = SD(k);

        %Add the SD as a patch if the user asked for this
        if strcmp(style,'patch') 
            h(k).sdPtch=patchMaker(SD(k),[0.6,0.6,1]);
        end

        %Build patch surfaces for SEM, the means, and optionally the medians
        if strcmp(style,'patch') || strcmp(style,'sdline')
            h(k).semPtch=patchMaker(SEM(k),[1,0.6,0.6]);
            h(k).mu=plot([X(k)-jitScale,X(k)+jitScale],[mu(k),mu(k)],'-r',...
                'linewidth',2);
            if markMedian
                statsOut(k).median = med(k);
                h(k).med=plot([X(k)-jitScale,X(k)+jitScale],[med(k),med(k)],':r',...
                    'linewidth',2);
            end
        end

        % Generate scatter in X
        thisX=violaPoints(thisX,thisY);
        C=cols(k,:);

        h(k).data=plot(thisX, thisY, 'o', 'color', C,...
                       'markerfacecolor', C+(1-C)*0.65);
    end  %for k=1:length(X)


    %Plot SD as a line
    if strcmp(style,'line') || strcmp(style,'sdline')
        for k=1:length(X)
            h(k).sd=plot([X(k),X(k)],[mu(k)-SD(k),mu(k)+SD(k)],...
                      '-','color',[0.2,0.2,1],'linewidth',2);
            set(h(k).sd,'ZData',[1,1]*-1)
        end
    end


    %Plot mean and SEM as a line, the means, and optionally the medians
    if strcmp(style,'line')
        for k=1:length(X)

            h(k).mu=plot(X(k),mu(k),'o','color','r',...
                'markerfacecolor','r',...
                'markersize',10);

            h(k).sem=plot([X(k),X(k)],[mu(k)-SEM(k),mu(k)+SEM(k)],'-r',...
                'linewidth',2);   
            if markMedian
                h(k).med=plot(X(k),med(k),'s','color',[0.8,0,0],...
                'markerfacecolor','none',...
                'lineWidth',2,...
                'markersize',12);
            end

             h(k).xAxisLocation=x(k);
        end
    end % if strcmp(style,'line')

    for thisInterval=1:length(h)
        h(thisInterval).interval=interval;
    end



    function ptch=patchMaker(thisInterval,tColor)
        %This nested function builds a patch for the SD or SEM
        l=mu(k)-thisInterval;
        u=mu(k)+thisInterval;
        ptch=patch([X(k)-jitScale, X(k)+jitScale, X(k)+jitScale, X(k)-jitScale],...
                [l,l,u,u], 0);
        set(ptch,'edgecolor',tColor*0.8,'facecolor',tColor)
    end %function patchMaker


    function X=violaPoints(X,Y)
        % Variable jitter according to how many points occupy each range of values. 
        [counts,~,bins] = histcounts(Y,10);
        inds = find(counts~=0);
        counts = counts(inds);

        Xr = X;
        for jj=1:length(inds)
            tWidth = jitter * (1-exp(-0.1 * (counts(jj)-1)));
            xpoints = linspace(-tWidth*0.8, tWidth*0.8, counts(jj));
            Xr(bins==inds(jj)) = xpoints;
        end
        X = X+Xr;
    end % function violaPoints


end % function myPlotter




end %function notBoxPlot
